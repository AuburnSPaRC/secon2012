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


//#define CALIBRATE_MOTORS   // Run in motor calibration mode.
//#define CALIBRATE_ENCODERS  // Run in encoder calibration mode.
//#define DEBUG_MOTOR_SPEED  // Display motor speeds in Serial window.
//#define DEBUG_LINE         // Display line sensor data
//#define DEBUG_COURSE       // Display info on robot's course location

#define FTA_CYCLE_SIZE  10 // Defines the size (in bytes) of the ftaCycle struct.
#define NUM_SETGMENTS  38 //The number of segments on the course
#define NUM_SENSORS   8     // number of sensors used on each array
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define MID_LINE      3500  // value of sensor when line is centered (0-7000)
#define WHITE_LINE    1     // '1' = white line, '0' = black line

#define REFLECT_THRESHOLD 750  // part of 1000 at which line is not found

// Direction Definitions
#define LEFT    true    // Left direction 
#define RIGHT   false   // Right direction

// Movement Actions
#define STOP                0 // Bot completely stops
#define TURN_LEFT           1 // Bot turns ~90 degrees left
#define TURN_RIGHT          2 // Bot turns ~90 degrees right
#define MOVE_FORWARD        3 // Bot moves forward at FULL_SPEED
#define PIVOT_LEFT_FORWARD  4 // Bot pivots forward on left wheel
#define PIVOT_LEFT_BACK     5 // Bot pivots back on left wheel
#define PIVOT_RIGHT_FORWARD 6 // Bot pivots forward on right wheel
#define PIVOT_RIGHT_BACK    7 // Bot pivots back on right wheel
#define MOVE_BACKWARD       8 // Bot moves backwards at FULL_SPEED

#define MAX_VELOCITY  255  // Maximum motor velocity

//#define TURN_TIME     570  // Number of mSeconds to turn robot 90 degrees
#define TURN_STOPS    10   // Number of encoder stops for a bot 90 degree turn

#define TL_NEGONE_COUNT 200
#define TL_ZERO_COUNT   200
#define TL_ONE_COUNT    200
#define TL_TWO_COUNT    200

#define ML_ZERO_COUNT   200
#define ML_THREE_COUNT  400
#define ML_FOUR_COUNT   400
#define ML_SEVEN_COUNT  400

// Maximum difference in wheel speeds when line-following
#define MAX_PID_DELTA (1-FULL_SPEED)*MAX_VELOCITY  
// PWM offset for motor speeds to be equal (Left motor is faster = +)
#define MOTOR_OFFSET  0

union u_double
{
  byte b[4];
  float dval;
  int ival;
};  //A structure to read in floats from the serial ports
  
  
// Container for the "follow, terminate, action" cycle values.
struct ftaCycle{
  byte follow;      // "Line-following" vs. "simple encoder travel"
  byte terminate;   // "At right turn", "at left turn", "off the line", etc.
  byte action;      // "turn in place", "turn left wheel then right", "turn right wheel then left"
  int leftAmount;   // Number of encoder clicks to turn left wheel forward (negative is backward)
  int rightAmount;  // Number of encoder clicks to turn right wheel forward (negative is backward)
  byte bot_speed;
  byte turn_speed;
  byte center;
};

float FULL_SPEED = 0.4; // Fraction of MAX_VELOCITY that is 'full' speed 
float TURN_SPEED = 0.25; // Fraction of MAX_VELOCITY that is 'turning' speed 

// Course Location (as defined by course_define.jpeg)
int location = 0; 

// Determines whether to go left or right after measurement
boolean goLeft = false;

// Initial PID Coeffs
double KP = .015; //.015
double KI = .0001; //.0001
double KD = .001; //.001

// Movement speeds
double leftDelta = 0;
double rightDelta = 0;
double forwardSpeed = 0;

int delayCounter = 0;

// Change these pins when you need to
unsigned char fSensorPinsRight[] = {33,35,37,39,41,43,45,47};
unsigned char fSensorPinsLeft[] = {13,12,11,10,9,8,7,6};//,9,10,11,12,13};
unsigned char lEncoderPins[] = {46};
unsigned char rEncoderPins[] = {48};

//unsigned char rSensorPins[] = {2,3,4,5,6,7,8,9};
#define LEFT_PWM_PIN   5
#define LEFT_DIR_PIN   38
#define RIGHT_PWM_PIN  12
#define RIGHT_DIR_PIN  13
//#define CAL_LED_PIN    5
#define RELAY_K1_PIN   52
#define RELAY_K2_PIN   53
//#define TEST_LED_PIN   51
#define LEFT_ENC_PIN   36
#define RIGHT_ENC_PIN  50


// Sensors 0 through 7 are connected to fSensorPins[]
PololuQTRSensorsRC fSensorRight(fSensorPinsRight, NUM_SENSORS, TIMEOUT); 
PololuQTRSensorsRC fSensorLeft(fSensorPinsLeft, NUM_SENSORS, TIMEOUT); 
PololuQTRSensorsRC lEncoder(lEncoderPins, 1, TIMEOUT);//, LEFT_ENC_PIN);
PololuQTRSensorsRC rEncoder(rEncoderPins, 1, TIMEOUT);//, RIGHT_ENC_PIN); 
unsigned int fSensorValuesRight[NUM_SENSORS];
unsigned int fSensorValuesLeft[NUM_SENSORS];
unsigned int fSensorValuesBoth[NUM_SENSORS*2];
unsigned int lEncoderValues[1];
unsigned int rEncoderValues[1];


// Setup PID computation
double setpointPID, inputPID, outputPID;
PID lfPID(&inputPID, &outputPID, &setpointPID, KP, KI, KD, DIRECT);

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
  //pinMode(TEST_LED_PIN, OUTPUT);
  
  //digitalWrite(TEST_LED_PIN, LOW);

  delay(50);
  
  digitalWrite(RELAY_K1_PIN, 0);
  digitalWrite(RELAY_K2_PIN, 0);
  
  motorCalibrate();          // Does nothing ifdef CALIBRATE_MOTORS
  
  setpointPID = MID_LINE;    // Set the point we want our PID loop to adjust to
   
  readPID(); // Read in the PID values from EEPROM 
  lfPID.SetTunings(KP, KI, KD); // Set the PID loops tuning values to the new ones from EEPROM
  lfPID.SetOutputLimits(-MAX_PID_DELTA, MAX_PID_DELTA); // force PID to the range of motor speeds.   
  lfPID.SetMode(AUTOMATIC);  // turn on the PID
  
  calibrateSensors(); // Calibrate line sensors
  
  delay(5000);
  
  // Start movement (starting at location defined in EEPROM)
  location=(int)EEPROM.read(421);
//  Serial.print("Location:");
//  Serial.print(location);
//  Serial.print("\n");
 
}

void loop()
{ 
  dynamic_PID();
  takeReading();
  executeSegment(location);
  increaseLocation();
  Serial.print("Location: ");
  Serial.print(location);
  Serial.print("\n");
 //followLine();
}

void dynamic_PID() // Sets the PID coefficients dynamically via a serial command interface...
{
  char command;			//The command coming in
  int i,j;			//Looping variables
  u_double Vals[3];		//Holds floating or double variables recieved
  
   //Variables to hold possible incoming data
  char follow,terminate,action;     
  int left_amnt,right_amnt;
  float p_val,i_val,d_val;
  int segment;
  byte b_speed,center,turn_speed,start_pos;
  //Debugging purposes ftaCycle currentSegment;
  //Debugging purposes byte tempMSB,tempLSB;
  if(Serial.available())		//Are there messages coming in?
  {
    delay(100);				//Wait a tad to let them in
      command=Serial.read();		//Read the command prefix
      switch (command)			
      {
        case 'p':			//Just a test command
          Serial.print("Print!");
          break;
        case 'c':                      //Course Variables
          if(Serial.available()>=9)	//We are getting three floats so wait til we get all their data
          {
            follow=Serial.read();
            terminate=Serial.read();
            action=Serial.read();
            Serial.print(follow);
            Serial.print("\n");
            Serial.print(terminate);
            Serial.print("\n");
            Serial.print(action);
            Serial.print("\n");        //For debugging purposes
            for(i=0;i<3;i++)		//Then read it in
            {
              for(j=0;j<2;j++)
              {
                Vals[i].b[j]=Serial.read();
              }
            }
            segment=Vals[0].ival;
            left_amnt=Vals[1].ival;
            right_amnt=Vals[2].ival;
            b_speed=Serial.read();
            turn_speed=Serial.read();
            center=Serial.read();
            Serial.print(segment);  //For Debugging Purposes
            Serial.print("\n");
            Serial.print((int)left_amnt);
            Serial.print("\n");
            Serial.print((int)right_amnt);
            Serial.print("\n");  
            Serial.print((int)b_speed);
            Serial.print("\n"); 
            Serial.print((int)turn_speed);
            Serial.print("\n");       
            Serial.print((int)center);
            Serial.print("\n");              
            
          }
          break;
        
        case 'g':			//Global variables
          if(Serial.available()>=13)	//We are getting three floats and an int so wait til we get all their data
          {
            for(i=0;i<3;i++)		//Then read it in
            {
              for(j=0;j<4;j++)
              {
                Vals[i].b[j]=Serial.read();
              }

            }
            start_pos=Serial.read();
            Serial.print("Starting:");
            Serial.print("\n");		//For now, we want to read them back out to make sure it worked
            Serial.print(start_pos);    //Debugging
            Serial.print("\n");Serial.flush();
          }
          break;
       }
       
       //Now store the values in EEProm
      switch (command)			
      {
        case 'c': 
        Serial.print("Here we are saving stuff!");
        Serial.print(segment*FTA_CYCLE_SIZE); 
          //Save the course variables into EEProm (the -48 is to go from char to byte values)
          EEPROM.write(segment * FTA_CYCLE_SIZE,follow-48);          
          EEPROM.write((segment * FTA_CYCLE_SIZE) + 1,terminate-48);
          EEPROM.write((segment * FTA_CYCLE_SIZE) + 2, action-48);
          EEPROM.write((segment * FTA_CYCLE_SIZE) + 3, Vals[1].b[1]);
          EEPROM.write((segment * FTA_CYCLE_SIZE) + 4, Vals[1].b[0]);
          EEPROM.write((segment * FTA_CYCLE_SIZE) + 5, Vals[2].b[1]);
          EEPROM.write((segment * FTA_CYCLE_SIZE) + 6, Vals[2].b[0]);
          EEPROM.write((segment * FTA_CYCLE_SIZE) + 7, b_speed);
          EEPROM.write((segment * FTA_CYCLE_SIZE) + 8, turn_speed);
          EEPROM.write((segment * FTA_CYCLE_SIZE) + 9, center);
          
          /*
          currentSegment.follow = EEPROM.read(segment * FTA_CYCLE_SIZE);          // Read the follow type
          currentSegment.terminate = EEPROM.read((segment * FTA_CYCLE_SIZE) + 1); // Read the termination type
          currentSegment.action = EEPROM.read((segment * FTA_CYCLE_SIZE) + 2);    // Read the action type
          tempMSB = EEPROM.read((segment * FTA_CYCLE_SIZE) + 3); // Read MSB of leftAmount
          tempLSB = EEPROM.read((segment * FTA_CYCLE_SIZE) + 4); // Read LSB of leftAmount
          currentSegment.leftAmount = word(tempMSB,tempLSB);     // Combine MSB and LSB
          tempMSB = EEPROM.read((segment * FTA_CYCLE_SIZE) + 5); // Read MSB of rightAmount
          tempLSB = EEPROM.read((segment * FTA_CYCLE_SIZE) + 6); // Read LSB of rightAmount
          currentSegment.rightAmount = word(tempMSB,tempLSB);    // Combine MSB and LSB
      
      
          Serial.print((currentSegment.follow));
          Serial.print("\n");
          Serial.print((currentSegment.terminate));
          Serial.print("\n");       
          Serial.print((currentSegment.action));
          Serial.print("\n");
          Serial.print((currentSegment.leftAmount));
          Serial.print("\n");   
          Serial.print((currentSegment.rightAmount));
          Serial.print("\n");*/   //Debugging print out
        break;
        
        case 'g':
          EEPROM.write(421,(start_pos));
          int address = 422;
          for (i = 0; i<3; ++i)
          {
            for (j = 0; j<4; ++j)
            {
              EEPROM.write(address+j, Vals[i].b[j]);
            
            }
             // Serial.print("\n");		//For now, we want to read them back out to make sure it worked
             // Serial.println(Vals[i].dval,5);      //Debugging
             // Serial.print("\n");Serial.flush(); 
            address += 4;
          }
        break;
      }
  }
}

// Reads in the PID values from EEPROM
// Note: "OOooOOOOOooOOH, scary pointer stuff!"
void readPID()
{
  u_double Val[3];
  int address = 422; // Starting address in EEPROM
  for (int i=0; i<3 ; ++i)
  {
    for (int j=0; j<4; ++j) // Read in the parts
    {
      Val[i].b[j]=EEPROM.read(address+j);
    }
    address+=4;    
  }
  
  KP = (float)Val[0].dval;
  KI = (float)Val[1].dval;
  KD = (float)Val[2].dval;
  
  Serial.print("KP: ");
  Serial.println(KP,5);
  Serial.print("KI: ");
  Serial.println(KI,5);
  Serial.print("KD: ");
  Serial.println(KD,5);
}
//Take a reading from the task sensors and makes L/R decision 
void takeReading()
{
 /* switch (location)
  {
    case 1: // Voltage Task
      goLeft = readVoltage();
      break;
    case 2: // Capacitance Task 
      goLeft = readCapacitance();
      break;
    case 5: // Temperature Task
      goLeft = readTemperature();
      break;
    case 6: // Waveform Task
      goLeft = readWaveform();
      break;
    default:
      goLeft = false; // Not at a task location.
      break;*/
  //}
  digitalWrite(RELAY_K1_PIN, 0);
  digitalWrite(RELAY_K2_PIN, 0);
}

void increaseLocation()
{
 if (goLeft)
    location += 3;
 if((location==6)||(location==15)||(location==26)||(location==35))
 {
   location+=3;
 }
  ++location;    // Increment location
  location %= 38; // Make sure location is never > 37
}

void calibrateSensors()
{
  boolean toggle = true;
  setMove(TURN_LEFT);
  // Calibrate sensors  (robot must be fully on the line)
  // Note: still needs calibration motor routine
  for (int i = 0; i < 25; i++)  // Make the calibration take about 5 seconds
  {
    // Reads both sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
    fSensorRight.calibrate(QTR_EMITTERS_ON);
    fSensorLeft.calibrate(QTR_EMITTERS_ON);
    lEncoder.calibrate(QTR_EMITTERS_ON);
    rEncoder.calibrate(QTR_EMITTERS_ON);
    digitalWrite(RELAY_K1_PIN, toggle); // Make sound!
    toggle = !toggle;
    //rSensor.calibrate();
  }
  
  toggle = true;
  setMove(TURN_RIGHT);
  for (int i = 0; i < 25; i++)  // Make the calibration take about 5 seconds
  {
    // Reads both sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
    fSensorRight.calibrate(QTR_EMITTERS_ON);
    fSensorLeft.calibrate(QTR_EMITTERS_ON);
    lEncoder.calibrate(QTR_EMITTERS_ON);
    rEncoder.calibrate(QTR_EMITTERS_ON);
    digitalWrite(RELAY_K1_PIN, toggle); // Make sound!
    toggle = !toggle;
    //rSensor.calibrate();
  }  
  setMove(STOP);
}


