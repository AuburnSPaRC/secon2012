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
#define REFLECT_THRESHOLD 250  // part of 1000 at which line is not found

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

  int type;      //INTERNAL USE ONLY     0 //NO BOX AROUND   1 //BOX IS TO ROBOT'S LEFT   2  // BOX IS TO ROBOT'S RIGHT
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

union u_int
{
  byte b[2];
  unsigned int ival;
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


//Calibrations stuff/
unsigned int fCalMax[16];
unsigned int fCalMin[16];
unsigned int encMax[1];
unsigned int encMin[1];
///////////////////////////


void setup()
{
  int calval=123;

  Serial.begin(9600);        // Begin serial comm for debugging  
  delay(100);                // Wait for serial comm to come online

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

  if((digitalRead(TOP_RIGHT_SWITCH)==LOW)&&(digitalRead(TOP_LEFT_SWITCH)==LOW)){
    EEPROM.write(1029,123);
  }
  if((digitalRead(BOTTOM_RIGHT_SWITCH)==LOW)&&(digitalRead(BOTTOM_LEFT_SWITCH)==LOW)){
    EEPROM.write(1029,321);
  }

  delay(50);

  digitalWrite(RELAY_K1_PIN, 0);
  digitalWrite(RELAY_K2_PIN, 0);

  delay(50);

  calval=EEPROM.read(1029);

  lfPID.SetSampleTime(10);

  if(calval==123)
  {
    calibrateSensors(); // Calibrate line sensors
    writeCalibrations();
    readCalibrations();
  }
  else 
  {   
    fSensors.calibrate(QTR_EMITTERS_ON);
    encoders.calibrate(QTR_EMITTERS_ON);
    readCalibrations();
    digitalWrite(RELAY_K1_PIN, 0);
    digitalWrite(RELAY_K2_PIN, 0);
  }


  delay(200);

  readConfigs(0, NUM_SEGMENTS);
  cur_loc=(int)EEPROM.read(1028);

}


void writeCalibrations(void)
{
  u_int maxval,minval;
  int i, j;

  // Serial.println("Writing values!");

  for(int i=0;i<16;i++)
  { 
    maxval.ival=fSensors.calibratedMaximumOn[i];
    minval.ival=fSensors.calibratedMinimumOn[i];
    for(int j=0;j<2;j++)
    {
      EEPROM.write(1030+(i*2)+j,maxval.b[j]);
      EEPROM.write(1062+(i*2)+j,minval.b[j]);
    }
    //        Serial.println(fSensors.calibratedMaximumOn[i]);
    //  Serial.println(fSensors.calibratedMinimumOn[i]);
    // Serial.print("\n\n");    
  }

  for(int i=0;i<2;i++)
  { 
    maxval.ival=encoders.calibratedMaximumOn[i];
    minval.ival=encoders.calibratedMinimumOn[i];
    for(int j=0;j<2;j++)
    {
      EEPROM.write(1094+(i*2)+j,maxval.b[j]);
      EEPROM.write(1098+(i*2)+j,minval.b[j]);
    }
    //      Serial.println(encoders.calibratedMaximumOn[i]);
    //  Serial.println(encoders.calibratedMinimumOn[i]);
    // Serial.print("\n\n");
  }


}


void readCalibrations(void)
{
  u_int maxval,minval;
  int i, j;

  //  Serial.println("Reading values!");
  for(int i=0;i<16;i++)
  { 
    for(int j=0;j<2;j++)
    {
      maxval.b[j]=EEPROM.read(1030+(i*2)+j);
      minval.b[j]=EEPROM.read(1062+(i*2)+j);
    }
    fSensors.calibratedMaximumOn[i]=maxval.ival;
    fSensors.calibratedMinimumOn[i]=minval.ival;

    /*  Serial.println(maxval.ival);
     Serial.println(minval.ival);
     Serial.print("\n\n");*/

  }

  for(int i=0;i<2;i++)
  { 
    for(int j=0;j<2;j++)
    {
      maxval.b[j]=EEPROM.read(1094+(i*2)+j);
      minval.b[j]=EEPROM.read(1098+(i*2)+j);
    }
    encoders.calibratedMaximumOn[i]=maxval.ival;
    encoders.calibratedMinimumOn[i]=minval.ival;

    /*    Serial.println(maxval.ival);
     Serial.println(minval.ival);
     Serial.print("\n\n");    */
  }
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
    delay(100);
    setMove(STOP);
    getData();
    setMove(MOVE_FORWARD);
    delay(100);
    setMove(STOP); 
  }

  if(!courseConfig[cur_loc].skip_section)    //Make sure we're not supposed to skip this section
  {
    takeReading();
    executeSegment(cur_loc);      //Carry on with current segment
  }
  else {
    Serial.print("Skipped that!\n");
  }

  moveOn();                     //Move to next location
}


void moveOn()
{
  if(goLeft){
    cur_loc+=4;
  }
  else if((cur_loc==6)||(cur_loc==15)||(cur_loc==26)||(cur_loc==35)){
    cur_loc+=4;
  }  
  else {
    cur_loc++;
  }
  if(cur_loc>39)cur_loc=0;
}

void takeReading(void)
{

  setMove(MOVE_FAST);
  delay(50);  

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
  setMove(STOP);
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
    if(i==1||i==10||i==19||i==21||i==30||i==39)courseConfig[i].type=0;  ///BOX is not around
    else if(i==7||i==8||i==9||i==16||i==17||i==18||i==27||i==28||i==29||i==36||i==37||i==38||i==20||i==0)courseConfig[i].type=2;    //BOX is to robot's right side
    else courseConfig[i].type=1;    //BOX is to robot's left side

    // --- Retrieve FTA information from EEPROM: ---
    courseConfig[i].follow = EEPROM.read(i * FTA_CYCLE_SIZE);          // Read the follow type
    courseConfig[i].termination = EEPROM.read((i * FTA_CYCLE_SIZE) + 1); // Read the termination type
    courseConfig[i].action = EEPROM.read((i * FTA_CYCLE_SIZE) + 2);    // Read the action type

    temp = EEPROM.read((i * FTA_CYCLE_SIZE) + 3); // Read leftAmount
    if(temp>127){
      courseConfig[i].leftAmount = int(-1*(256-temp));
    }
    else courseConfig[i].leftAmount=int(temp);

    temp = EEPROM.read((i * FTA_CYCLE_SIZE) + 4); // Read rightAmount
    if(temp>127){
      courseConfig[i].rightAmount = int(-1*(256-temp));
    }
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
  boolean toggle = true;

  TURN_SPEED=0.3;
  setMove(TURN_LEFT);
  // Calibrate sensors  (robot must be fully on the line)
  // Note: still needs calibration motor routine
  for (int i = 0; i < 40; i++)  // Make the calibration take about 5 seconds
  {
    // Reads both sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
    fSensors.calibrate(QTR_EMITTERS_ON);
    encoders.calibrate(QTR_EMITTERS_ON);
    if(i%3==0)
    {
      digitalWrite(RELAY_K1_PIN, toggle); // Make sound!
      toggle = !toggle;
    }
  }

  toggle = true;
  setMove(TURN_RIGHT);
  for (int i = 0; i < 40; i++)  // Make the calibration take about 5 seconds
  {
    // Reads both sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
    fSensors.calibrate(QTR_EMITTERS_ON);
    encoders.calibrate(QTR_EMITTERS_ON);
    if(i%3==0)
    {
      digitalWrite(RELAY_K1_PIN, toggle); // Make sound!
      toggle = !toggle;
    }
  }
  setMove(STOP);
  TURN_SPEED=temp;
  digitalWrite(RELAY_K1_PIN, 0);
  digitalWrite(RELAY_K2_PIN, 0);
}




