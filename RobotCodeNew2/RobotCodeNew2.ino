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


#define FTA_CYCLE_SIZE  23 // Defines the size (in bytes) of the ftaCycle struct.

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

float FULL_SPEED = 0.4; // Fraction of MAX_VELOCITY that is 'full' speed, set each segment
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
  byte skip_section;
};

//A global variable used to store the current termination
int atTermination;

ftaCycle courseConfig[NUM_SEGMENTS];     //Holds the settings for the course read in from EEProm 
boolean goLeft=0;                        //Whether or not we have read the box as going left

//Structure to read in data fromt the serial port into ints and floats
union u_double
{
  byte b[4];
  float dval;
  int ival;
};  //A structure to read in floats from the serial ports

int cur_loc=0;    //Our current location on the course
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
PID lfPID(&inputPID, &outputPID, &setpointPID, 0.023,0.0,0.008, DIRECT);
///////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(9600);        // Begin serial comm for debugging  
  delay(500);                // Wait for serial comm to come online

  // Setup pin IO:
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  pinMode(RELAY_K1_PIN, OUTPUT);
  pinMode(RELAY_K2_PIN, OUTPUT);
  pinMode(TOP_RIGHT_SWITCH, INPUT);
  pinMode(BOTTOM_RIGHT_SWITCH, INPUT);
  pinMode(TOP_LEFT_SWITCH, INPUT);
  pinMode(BOTTOM_LEFT_SWITCH, INPUT);
  

  delay(50);
  
    digitalWrite(RELAY_K1_PIN, 0);
  digitalWrite(RELAY_K2_PIN, 0);
  
  delay(50);
  
  lfPID.SetSampleTime(10);

  calibrateSensors(); // Calibrate line sensors

  delay(1000);
  
  readConfigs(0, NUM_SEGMENTS);
  cur_loc=2;//(int)EEPROM.read(1028);
}


void loop(void)
{
/*  Serial.print("\n");
  Serial.print("Location: ");
  Serial.print(cur_loc);
  Serial.print("\n"); */
 while(Serial.available()>0)    //First see if we have any incoming messages
 {
   Serial.println(Serial.available());
   setMove(MOVE_FORWARD);
   getData();
   delay(250);
   setMove(STOP); 
 }
 
 if(!courseConfig[cur_loc].skip_section)    //Make sure we're not supposed to skip this section
 {
   takeReading();
   executeSegment(cur_loc);      //Carry on with current segment
 }
 else {Serial.print("Skipped that!\n");}
 
 moveOn();                     //Move to next location
 //readTemperature();
 
}


void moveOn()
{
  if(goLeft){cur_loc+=4;}
  else if((cur_loc==6)||(cur_loc==15)||(cur_loc==26)||(cur_loc==35)){cur_loc+=4;}  
  else {cur_loc++;}
  if(cur_loc>39)cur_loc=0;
}

void takeReading(void)
{
   switch (cur_loc)
  {
  case 3: // Voltage Task
    goLeft = readVoltage();
    break;
  case 12: // Capacitance Task 
    readCapacitance();      //For some reason it does better if we do this....
    goLeft = readCapacitance();
    break;
  case 23: // Temperature Task
    goLeft = readTemperature();    
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



void getData(void)
{
  char command;
  int segment;
  int i,j;  
  byte start_pos, leftAmount, rightAmount;
  u_double Vals[3];
  
  
  command=Serial.read();
  Serial.println(command);
  switch(command)
  {
    case 'p':			//Just a test command
      Serial.print("Print!");
      break;
    
    //Check to make sure we are sending it things
    case 'c':    //Data command
      delay(100);
      if(Serial.available()>=22)
      {
        //Get the data
        segment=Serial.read();
        courseConfig[segment].follow=Serial.read();
        courseConfig[segment].termination=Serial.read();            
        courseConfig[segment].action=Serial.read();    
        leftAmount=Serial.read();             
        rightAmount=Serial.read();      
        courseConfig[segment].bot_speed=Serial.read();
        courseConfig[segment].turn_speed=Serial.read();
        courseConfig[segment].center=Serial.read();
        courseConfig[segment].occurance=Serial.read();
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
        
        courseConfig[segment].skip_section=Serial.read();  
        start_pos=Serial.read(); 
     
     
        //Now save the data   
        EEPROM.write(segment * FTA_CYCLE_SIZE,courseConfig[segment].follow);          
        EEPROM.write((segment * FTA_CYCLE_SIZE) + 1,courseConfig[segment].termination);
        EEPROM.write((segment * FTA_CYCLE_SIZE) + 2, courseConfig[segment].action);
        EEPROM.write((segment * FTA_CYCLE_SIZE) + 3,leftAmount);          
        EEPROM.write((segment * FTA_CYCLE_SIZE) + 4,rightAmount);
        EEPROM.write((segment * FTA_CYCLE_SIZE) + 5, courseConfig[segment].bot_speed);          
        EEPROM.write((segment * FTA_CYCLE_SIZE) + 6, courseConfig[segment].turn_speed);          
        EEPROM.write((segment * FTA_CYCLE_SIZE) + 7,courseConfig[segment].center);
        EEPROM.write((segment * FTA_CYCLE_SIZE) + 8,courseConfig[segment].occurance);
        EEPROM.write((segment * FTA_CYCLE_SIZE) + 9,courseConfig[segment].clicks);

        //KP
        EEPROM.write((segment * FTA_CYCLE_SIZE) + 10, Vals[0].b[0]);
        EEPROM.write((segment * FTA_CYCLE_SIZE) + 11, Vals[0].b[1]);
        EEPROM.write((segment * FTA_CYCLE_SIZE) + 12, Vals[0].b[2]);
        EEPROM.write  ((segment * FTA_CYCLE_SIZE) + 13, Vals[0].b[3]);
    
        //KI
        EEPROM.write((segment * FTA_CYCLE_SIZE) + 14, Vals[1].b[0]);
        EEPROM.write((segment * FTA_CYCLE_SIZE) + 15, Vals[1].b[1]);
        EEPROM.write((segment * FTA_CYCLE_SIZE) + 16, Vals[1].b[2]);
        EEPROM.write((segment * FTA_CYCLE_SIZE) + 17, Vals[1].b[3]);
    
        //KD
        EEPROM.write((segment * FTA_CYCLE_SIZE) + 18, Vals[2].b[0]);
        EEPROM.write((segment * FTA_CYCLE_SIZE) + 19, Vals[2].b[1]);
        EEPROM.write((segment * FTA_CYCLE_SIZE) + 20, Vals[2].b[2]);
        EEPROM.write((segment * FTA_CYCLE_SIZE) + 21, Vals[2].b[3]);
        
        //Skip this section?
        EEPROM.write((segment * FTA_CYCLE_SIZE) + 22, courseConfig[segment].skip_section);
        
        //Start Position
        EEPROM.write(1028,(start_pos));
        
        readConfigs(segment,segment+1);
        Serial.println("We got it!");

      }
      break;
      
  }
  
}




void readConfigs(int startRead, int stopRead)
{
  int temp=0;
  u_double tempVals[3];    //Temporary structs to read in the floats
  for(int i=startRead;i<stopRead;i++)
  {
    // --- Retrieve FTA information from EEPROM: ---
    courseConfig[i].follow = EEPROM.read(i * FTA_CYCLE_SIZE);          // Read the follow type
    courseConfig[i].termination = EEPROM.read((i * FTA_CYCLE_SIZE) + 1); // Read the termination type
    courseConfig[i].action = EEPROM.read((i * FTA_CYCLE_SIZE) + 2);    // Read the action type
    
    temp = EEPROM.read((i * FTA_CYCLE_SIZE) + 3); // Read leftAmount
    if(temp>127){courseConfig[i].leftAmount = int(-1*(256-temp));}
    else courseConfig[i].leftAmount=int(temp);
  
     temp = EEPROM.read((i * FTA_CYCLE_SIZE) + 4); // Read rightAmount
    if(temp>127){courseConfig[i].rightAmount = int(-1*(256-temp));}
    else courseConfig[i].rightAmount=int(temp);
    
    courseConfig[i].bot_speed = EEPROM.read((i * FTA_CYCLE_SIZE) + 5); // Read bot_speed
    courseConfig[i].turn_speed = EEPROM.read((i * FTA_CYCLE_SIZE) + 6); // Read turn speed  
    courseConfig[i].center = EEPROM.read((i * FTA_CYCLE_SIZE) + 7); // Read center of line
    courseConfig[i].occurance = EEPROM.read((i * FTA_CYCLE_SIZE) + 8); // Read occurances of end action
    courseConfig[i].clicks = EEPROM.read((i * FTA_CYCLE_SIZE) + 9); // Read clicks to travel before end action
      
    
    //Read in the KP, KI, KD values 
    tempVals[0].b[0]=EEPROM.read((i * FTA_CYCLE_SIZE) + 10);
    tempVals[0].b[1]=EEPROM.read((i * FTA_CYCLE_SIZE) + 11);  
    tempVals[0].b[2]=EEPROM.read((i * FTA_CYCLE_SIZE) + 12);
    tempVals[0].b[3]=EEPROM.read((i * FTA_CYCLE_SIZE) + 13);  
  
    tempVals[1].b[0]=EEPROM.read((i * FTA_CYCLE_SIZE) + 14);
    tempVals[1].b[1]=EEPROM.read((i * FTA_CYCLE_SIZE) + 15);  
    tempVals[1].b[2]=EEPROM.read((i * FTA_CYCLE_SIZE) + 16);
    tempVals[1].b[3]=EEPROM.read((i * FTA_CYCLE_SIZE) + 17);  

    tempVals[2].b[0]=EEPROM.read((i * FTA_CYCLE_SIZE) + 18);
    tempVals[2].b[1]=EEPROM.read((i * FTA_CYCLE_SIZE) + 19);  
    tempVals[2].b[2]=EEPROM.read((i * FTA_CYCLE_SIZE) + 20);
    tempVals[2].b[3]=EEPROM.read((i * FTA_CYCLE_SIZE) + 21);
  
    courseConfig[i].KP=tempVals[0].dval;
    courseConfig[i].KI=tempVals[1].dval;
    courseConfig[i].KD=tempVals[2].dval;

    courseConfig[i].skip_section = EEPROM.read((i * FTA_CYCLE_SIZE) + 22); // Read clicks to travel before end action       
  }
}



void calibrateSensors()
{
  float temp=TURN_SPEED;
  
  TURN_SPEED=0.3;
  setMove(TURN_LEFT);
  // Calibrate sensors  (robot must be fully on the line)
  // Note: still needs calibration motor routine
  for (int i = 0; i < 50; i++)  // Make the calibration take about 5 seconds
  {
    // Reads both sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
    fSensors.calibrate(QTR_EMITTERS_ON);
    encoders.calibrate(QTR_EMITTERS_ON);
    //rEncoder.calibrate(QTR_EMITTERS_ON);
  }

  setMove(TURN_RIGHT);
  for (int i = 0; i < 50; i++)  // Make the calibration take about 5 seconds
  {
    // Reads both sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
    fSensors.calibrate(QTR_EMITTERS_ON);
    encoders.calibrate(QTR_EMITTERS_ON);
  //  rEncoder.calibrate(QTR_EMITTERS_ON);
  }
  setMove(STOP);
  TURN_SPEED=temp;
}



