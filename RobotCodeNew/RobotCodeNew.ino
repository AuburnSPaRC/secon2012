/** Main code for operation of SECON robot.
*
* Includes navigation system and line-following system, as well as calibration
* and setup routines. For reference, high values from line sensor are black,
* low are white.
*
* Depends: MotionFunctions.ino
* MotorFunctions.ino
* TaskSensorFunctions.ino
* PID_v1.h
* PololuQTRSensors.h
*
* Credits: Tyler Crumpton
*
*/
#include <PID_v1.h>
#include <PololuQTRSensors.h>
#include <EEPROM.h>
#include "PinDefines.h"

<<<<<<< HEAD
#define FTA_CYCLE_SIZE 20 // Defines the size (in bytes) of the ftaCycle struct.
#define NUM_SEGMENTS 40 //The number of segments on the course
#define NUM_SENSORS 16 // number of sensors used on each array
#define TIMEOUT 2500 // waits for 2500 us for sensor outputs to go low
#define MID_LINE ((NUM_SENSORS-1)*1000)/2 // value of sensor when line is centered (0-7000)
#define WHITE_LINE 1 // '1' = white line, '0' = black line
#define REFLECT_THRESHOLD 650 // part of 1000 at which line is not found
=======
#define FTA_CYCLE_SIZE  20 // Defines the size (in bytes) of the ftaCycle struct.
#define NUM_SEGMENTS  40 //The number of segments on the course
#define NUM_SENSORS   16    // number of sensors used on each array
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define MID_LINE      ((NUM_SENSORS-1)*1000)/2  // value of sensor when line is centered (0-7000)
#define WHITE_LINE    1     // '1' = white line, '0' = black line
#define REFLECT_THRESHOLD 650  // part of 1000 at which line is not found
>>>>>>> ef8898c689414ac2ad08beaf6b53c73d1c8c0d41


// Direction Definitions
#define LEFT true // Left direction
#define RIGHT false // Right direction

// Movement Actions
#define STOP 0 // Bot completely stops
#define TURN_LEFT 1 // Bot turns ~90 degrees left
#define TURN_RIGHT 2 // Bot turns ~90 degrees right
#define MOVE_FORWARD 3 // Bot moves forward at FULL_SPEED
#define PIVOT_LEFT_FORWARD 4 // Bot pivots forward on left wheel
#define PIVOT_LEFT_BACK 5 // Bot pivots back on left wheel
#define PIVOT_RIGHT_FORWARD 6 // Bot pivots forward on right wheel
#define PIVOT_RIGHT_BACK 7 // Bot pivots back on right wheel
#define MOVE_BACKWARD 8 // Bot moves backwards at FULL_SPEED
#define MOVE_FAST 9 // Bot moves at FULLSPEED

#define MAX_VELOCITY 255 // Maximum motor velocity

//#define FAST_TEMP
//#define DEBUG_ROBOT 0
//#define RE
//#define DEBUG_PID 0
//#define DEBUG_WAVEFORM
//#define DEBUG_CAPACITANCE
//#define DEBUG_VOLTAGE
//#define DEBUG_TEMP

// PWM offset for motor speeds to be equal (Left motor is faster = +)
#define MOTOR_OFFSET 0

//Structure to read in data fromt the serial port into ints and floats
union u_double
{
  byte b[4];
  float dval;
  int ival;
}; //A structure to read in floats from the serial ports

// Container for the "follow, terminate, action" cycle values.
struct ftaCycle{
  byte follow; // "Line-following" vs. "simple encoder travel"
  byte terminate; // "At right turn", "at left turn", "off the line", etc.
  byte action; // "turn in place", "turn left wheel then right", "turn right wheel then left"
  int leftAmount; // Number of encoder clicks to turn left wheel forward (negative is backward)
  int rightAmount; // Number of encoder clicks to turn right wheel forward (negative is backward)
  byte bot_speed;
  byte turn_speed;
  byte center;
  float KP;
  float KI;
  float KD;
};

ftaCycle courseConfig[NUM_SEGMENTS]; //Holds the settings for the course read in from EEProm

float MAX_PID_DELTA=0; // Maximum difference in wheel speeds when line-following

float FULL_SPEED = 0.4; // Fraction of MAX_VELOCITY that is 'full' speed, set each segment
float TURN_SPEED = 0.3; // Fraction of MAX_VELOCITY that is 'turning' speed, set each segment

// Course Location (as defined by course_define.jpeg)
int location = 0;

// Determines whether to go left or right after measurement
boolean goLeft = false;

// Movement speeds
double leftDelta = 0;
double rightDelta = 0;
double forwardSpeed = 0;

// Sensors 0 through 7 are connected to fSensorPins[]
PololuQTRSensorsRC fSensors(fSensorPins, NUM_SENSORS, TIMEOUT);
PololuQTRSensorsRC lEncoder(lEncoderPins, 1, TIMEOUT);//, LEFT_ENC_PIN);
PololuQTRSensorsRC rEncoder(rEncoderPins, 1, TIMEOUT);//, RIGHT_ENC_PIN);
unsigned int fSensorValues[NUM_SENSORS];
unsigned int lEncoderValues[1];
unsigned int rEncoderValues[1];




// Setup PID computation
double setpointPID, inputPID, outputPID;
PID lfPID(&inputPID, &outputPID, &setpointPID, 0.1,0.1,0.1, DIRECT);

void setup()
{
  Serial.begin(9600); // Begin serial comm for debugging
  delay(500); // Wait for serial comm to come online

  // Setup pin IO:
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  pinMode(RELAY_K1_PIN, OUTPUT);
  pinMode(RELAY_K2_PIN, OUTPUT);
  pinMode(HIT_SWITCH_PIN, INPUT);

  delay(50);
  
  lfPID.SetSampleTime(10);

  //Set up the relays
  digitalWrite(RELAY_K1_PIN, 0);
  digitalWrite(RELAY_K2_PIN, 0);

  calibrateSensors(); // Calibrate line sensors

    delay(2000);

  // Start movement (starting at location defined in EEPROM)
  location=(int)EEPROM.read(1028);
}

void loop()
{
  
#ifdef DEBUG_ROBOT
  Serial.print("\n");
  Serial.print("Location: ");
  Serial.print(location);
  Serial.print("\n");
#endif
 // readTemperature();
  
  

  if(Serial.available()){
    delay(1000);
    Serial.print("We got something!");
    dynamic_PID();
  }
#ifndef DEBUG_WAVEFORM
  takeReading();
  executeSegment(location);
  increaseLocation();
#endif
  
  #ifdef DEBUG_WAVEFORM
  readWaveform();
  #endif
  //followLine();
}

void dynamic_PID() // Sets the PID coefficients dynamically via a serial command interface...
{
  char command; //The command coming in
  int segment; //What stage we are reading in values for
  int i,j; //Looping variables
  byte start_pos,leftAmount,rightAmount; //New starting location
  u_double Vals[3];
  

  command=Serial.read(); //Read the command prefix
  switch (command)
  {
  case 'p': //Just a test command
    Serial.print("Print!");
    break;
  case 'c': //Course Variables
    if(Serial.available()>=20) //We are getting three floats so wait til we get all their data
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
      for(i=0;i<3;i++) //Read in PID values
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
    break;

  case 'g': //Global variables
    if(Serial.available()>=1) //We are getting three floats and an int so wait til we get all their data
    {
      start_pos=Serial.read();
    }
    break;
  }

  //Now store the values in EEProm
  switch (command)
  {
  case 'c':
  #ifdef DEBUG_PID
        setMove(STOP);
        lfPID.SetOutputLimits(-MAX_PID_DELTA, MAX_PID_DELTA);
        lfPID.SetTunings(double(Vals[0].dval), double(Vals[1].dval),double(Vals[2].dval)); // Set the PID loops tuning values to the new ones from EEPROM
        setpointPID=courseConfig[location].center*100;
        Serial.print("New Values set: ");
        Serial.println(lfPID.GetKp(),5);
        Serial.println(lfPID.GetKi(),5);
        Serial.println(lfPID.GetKd(),5);
        delay(1000);
  #endif
  #ifndef DEBUG_PID
    Serial.print("Here we are saving stuff!");
    Serial.print(segment*FTA_CYCLE_SIZE);
    //Save the course variables into EEProm (the -48 is to go from char to byte values)
    EEPROM.write(segment * FTA_CYCLE_SIZE,courseConfig[segment].follow);
    EEPROM.write((segment * FTA_CYCLE_SIZE) + 1,courseConfig[segment].terminate);
    EEPROM.write((segment * FTA_CYCLE_SIZE) + 2, courseConfig[segment].action);
    EEPROM.write((segment * FTA_CYCLE_SIZE) + 3,leftAmount);
    EEPROM.write((segment * FTA_CYCLE_SIZE) + 4,rightAmount);
    EEPROM.write((segment * FTA_CYCLE_SIZE) + 5, courseConfig[segment].bot_speed);
    EEPROM.write((segment * FTA_CYCLE_SIZE) + 6, courseConfig[segment].turn_speed);
    EEPROM.write((segment * FTA_CYCLE_SIZE) + 7,courseConfig[segment].center);

    //KP
    EEPROM.write((segment * FTA_CYCLE_SIZE) + 8, Vals[0].b[0]);
    EEPROM.write((segment * FTA_CYCLE_SIZE) + 9, Vals[0].b[1]);
    EEPROM.write((segment * FTA_CYCLE_SIZE) + 10, Vals[0].b[2]);
    EEPROM.write ((segment * FTA_CYCLE_SIZE) + 11, Vals[0].b[3]);

    //KI
    EEPROM.write((segment * FTA_CYCLE_SIZE) + 12, Vals[1].b[0]);
    EEPROM.write((segment * FTA_CYCLE_SIZE) + 13, Vals[1].b[1]);
    EEPROM.write((segment * FTA_CYCLE_SIZE) + 14, Vals[1].b[2]);
    EEPROM.write((segment * FTA_CYCLE_SIZE) + 15, Vals[1].b[3]);

    //KD
    EEPROM.write((segment * FTA_CYCLE_SIZE) + 16, Vals[2].b[0]);
    EEPROM.write((segment * FTA_CYCLE_SIZE) + 17, Vals[2].b[1]);
    EEPROM.write((segment * FTA_CYCLE_SIZE) + 18, Vals[2].b[2]);
    EEPROM.write((segment * FTA_CYCLE_SIZE) + 19, Vals[2].b[3]);
   #endif
    break;

  case 'g':
    EEPROM.write(1028,(start_pos));
    break;
  }
}


//Take a reading from the task sensors and makes L/R decision
void takeReading()
{
  switch (location)
  {
  case 3: // Voltage Task
    goLeft = readVoltage();
    break;
  case 12: // Capacitance Task
    readCapacitance(); //For some reason it does better if we do this....
    goLeft = readCapacitance();
    break;
  case 23: // Temperature Task
    goLeft = readTemperature();
    //goLeft = false; //For debugging
    break;
  case 32: // Waveform Task
    goLeft = readWaveform();
    break;
  default:
    goLeft = false; // Not at a task location.
    break;
  }
  digitalWrite(RELAY_K1_PIN, 0);
  digitalWrite(RELAY_K2_PIN, 0);
}


void increaseLocation()
{
  if(location==15){location=20;}
  else if(location==35){location=0;}
  else if(location==18){location=20;}
  else if(location==38){location=0;}
  else
  {
    if (goLeft){location += 4;}
    else if((location==6)||(location==15)||(location==26)||(location==35)){location+=4;}
    else location++;
    if(location>39)location=0; // Make sure location is never > 37
  }
}

void calibrateSensors()
{
  boolean toggle = true;
  setMove(TURN_LEFT);
  // Calibrate sensors (robot must be fully on the line)
  // Note: still needs calibration motor routine
  for (int i = 0; i < 30; i++) // Make the calibration take about 5 seconds
  {
    // Reads both sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
    fSensors.calibrate(QTR_EMITTERS_ON);
    lEncoder.calibrate(QTR_EMITTERS_ON);
    rEncoder.calibrate(QTR_EMITTERS_ON);
    if(i%2==0)
    {
     digitalWrite(RELAY_K1_PIN, toggle); // Make sound!
     toggle = !toggle;
    }
    //rSensor.calibrate();
  }

  toggle = true;
  setMove(TURN_RIGHT);
  for (int i = 0; i < 30; i++) // Make the calibration take about 5 seconds
  {
    // Reads both sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
    fSensors.calibrate(QTR_EMITTERS_ON);
    lEncoder.calibrate(QTR_EMITTERS_ON);
    rEncoder.calibrate(QTR_EMITTERS_ON);
    if(i%2==0)
    {
     digitalWrite(RELAY_K1_PIN, toggle); // Make sound!
     toggle = !toggle;
    }
    //rSensor.calibrate();
  }
  setMove(STOP);
}



