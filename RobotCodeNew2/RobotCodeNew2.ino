/**  Main code for operation of SECON robot.
 * 
 * Includes navigation system and line-following system, as well as calibration
 * and setup routines. For reference, high values from line sensor are black, 
 * low are white.
 *
 * Depends: MotionFunctions.ino
 *          MotorFunctions.ino
 *          TaskSensorFunctions.ino
 *          PID_v1.h
 *          PololuQTRSensors.h
 *
 * Credits: Tyler Crumpton
 *
 */
#include <PID_v1.h>
#include <PololuQTRSensors.h>
#include <EEPROM.h>
#include "PinDefines.h"


#define FTA_CYCLE_SIZE  22 // Defines the size (in bytes) of the ftaCycle struct.

#define NUM_SEGMENTS  40 //The number of segments on the course
#define NUM_SENSORS   16    // number of sensors used on each array
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define MID_LINE      ((NUM_SENSORS-1)*1000)/2  // value of sensor when line is centered (0-7000)
#define WHITE_LINE    1     // '1' = white line, '0' = black line
#define REFLECT_THRESHOLD 650  // part of 1000 at which line is not found

// Direction Definitions
#define LEFT    0    // Left direction 
#define RIGHT   1   // Right direction

// Movement Actions
#define STOP                0 // Bot completely stops
#define TURN_LEFT           1 // Bot turns ~90 degrees left
#define TURN_RIGHT          2 // Bot turns ~90 degrees right
#define MOVE_FORWARD        3 // Bot moves forward at FULL_SPEED
#define LEFT_FORWARD        4 // Bot left wheel pivots forward
#define LEFT_BACK           5 // Bot left wheel pivots back 
#define RIGHT_FORWARD       6 // Bot right wheel pivots forward
#define RIGHT_BACK          7 // Bot right wheel pivots back 
#define MOVE_BACKWARD       8 // Bot moves backwards at FULL_SPEED
#define MOVE_FAST           9 // Bot moves at FULLSPEED
#define STOP_LEFT_WHEEL     10 //Bot's left wheel stops
#define STOP_RIGHT_WHEEL    11 //Bot's right wheel stops


//MOTOR STUFF/////////////////////////////////////////////////////////////////////////
#define MAX_VELOCITY  255  // Maximum motor velocity
#define MOTOR_OFFSET  0

float FULL_SPEED = 0.8; // Fraction of MAX_VELOCITY that is 'full' speed, set each segment
float TURN_SPEED = 0.8; // Fraction of MAX_VELOCITY that is 'turning' speed, set each segment

float MAX_DELTA = FULL_SPEED*MAX_VELOCITY;

// Movement speeds
double leftDelta = 0;
double rightDelta = 0;
double forwardSpeed = 0;
////////////////////////////////////////////////////////////////////////////////////////


//SECTION STUFF//////////////////////////////////////////////////////////////////////////
// Container for the "follow, terminate, action" cycle values.
struct ftaCycle{
  byte follow;      // "Line-following" vs. "simple encoder travel"
  byte termination;   // "At right turn", "at left turn", "off the line", etc.
  byte action;      // "turn in place", "turn left wheel then right", "turn right wheel then left"
  int leftAmount;   // Number of encoder clicks to turn left wheel forward (negative is backward)
  int rightAmount;  // Number of encoder clicks to turn right wheel forward (negative is backward)
  byte bot_speed;
  byte turn_speed;
  byte center;
  byte occurance;
  byte clicks;
  float KP;
  float KI;
  float KD;
};

ftaCycle courseConfig[NUM_SEGMENTS];     //Holds the settings for the course read in from EEProm 
boolean goLeft=0;                        //Whether or not we have read the box as going left

//Structure to read in data fromt the serial port into ints and floats
union u_double
{
  byte b[4];
  float dval;
  int ival;
};  //A structure to read in floats from the serial ports

//////////////////////////////////////////////////////////////////////////////////////////////



//Sensor Stuff////////////////////////////////////////////////////////////////////////////
PololuQTRSensorsRC fSensors(fSensorPins, NUM_SENSORS, TIMEOUT); 
unsigned int fSensorValues[NUM_SENSORS];

PololuQTRSensorsRC turnSensors(turnSensorPins, 2, TIMEOUT);
unsigned int turnSensorValues[2];

PololuQTRSensorsRC encoders(encoderPins, 2, TIMEOUT);//, LEFT_ENC_PIN);
//PololuQTRSensorsRC rEncoder(rEncoderPins, 1, TIMEOUT);//, RIGHT_ENC_PIN); 
unsigned int encoderValues[2];
//unsigned int rEncoderValues[1];

// Setup PID computation
double setpointPID, inputPID, outputPID;
PID lfPID(&inputPID, &outputPID, &setpointPID, 0.0125,0.002,0.004, DIRECT);
///////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(9600);        // Begin serial comm for debugging  
  delay(500);                // Wait for serial comm to come online

  // Setup pin IO:

  delay(50);
  
  lfPID.SetSampleTime(10);

  calibrateSensors(); // Calibrate line sensors

  delay(1000);
  
  setpointPID=8000;
  lfPID.SetTunings(0.02,0.000,0.004);
  //lfPID.SetTunings(0.0185,0.0,0.0081667);
  lfPID.SetOutputLimits(-MAX_DELTA, MAX_DELTA); // force PID to the range of motor speeds.   
  lfPID.SetMode(AUTOMATIC); 
  lfPID.SetMode(MANUAL);
  lfPID.SetMode(AUTOMATIC);
}


void loop(void)
{
 executeSegment(0);
}





void getData(void)
{
  char command;
  int segment;
  int i,j;  
  byte start_pos, leftAmount, rightAmount;
  u_double Vals[3];
  
  
  command=Serial.read();
  switch(command)
  {
    case 'c':
      if(Serial.available()>=22)
      {
        segment=Serial.read();
        courseConfig[segment].follow=Serial.read();
        courseConfig[segment].terminate=Serial.read();            
        courseConfig[segment].action=Serial.read();    
        leftAmount=Serial.read();             
        rightAmount=Serial.read();      
        courseConfig[segment].bot_speed=Serial.read();
        courseConfig[segment].turn_speed=Serial.read();
        courseConfig[segment].center=Serial.read();
        courseConfig[segment].occurances=Serial.read();
        courseConfig[segment].clicks=Serial.read();
        for(i=0;i<3;i++)		//Read in PID values
        {
          for(j=0;j<4;j++)
          {
            Vals[i].b[j]=Serial.read();
          }
        }
        courseConfig[segment].KP=Vals[0].dval;
        courseConfig[segment].KI=Vals[1].dval;            
        courseConfig[segment].KD=Vals[2].dval;         
      }
  }
  
}






void calibrateSensors()
{
  float temp=TURN_SPEED;
  
  TURN_SPEED=0.35;
  setMove(TURN_LEFT);
  // Calibrate sensors  (robot must be fully on the line)
  // Note: still needs calibration motor routine
  for (int i = 0; i < 25; i++)  // Make the calibration take about 5 seconds
  {
    // Reads both sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
    fSensors.calibrate(QTR_EMITTERS_ON);
    //turnSensors.calibrate(QTR_EMITTERS_ON);
    encoders.calibrate(QTR_EMITTERS_ON);
    //rEncoder.calibrate(QTR_EMITTERS_ON);
  }

  setMove(TURN_RIGHT);
  for (int i = 0; i < 25; i++)  // Make the calibration take about 5 seconds
  {
    // Reads both sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
    fSensors.calibrate(QTR_EMITTERS_ON);
    //turnSensors.calibrate(QTR_EMITTERS_ON);
    encoders.calibrate(QTR_EMITTERS_ON);
  //  rEncoder.calibrate(QTR_EMITTERS_ON);
  }
  setMove(STOP);
  TURN_SPEED=temp;
}



