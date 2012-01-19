#include <PID_v1.h>
#include <PololuQTRSensors.h>

//    ---------    <== Front of bot sensor (fSensor)
//            |
//            |
//            |    <== Right of bot sensor (rSensor)
//            |

// For reference: high values from sensor are black, low are white.

#define NUM_SENSORS   8     // number of sensors used on each array
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define MID_LINE      3500  // value of sensor when line is centered (0-7000)
#define WHITE_LINE    1     // '1' = white line, '0' = black line

#define REFLECT_THRESHOLD 750  // part of 1000 at which line is not found

// Robot States
//#define ON_MAIN       0 // following the main line
//#define AT_BOX        1 // leaves from straight line to 'T'
//#define DECIDED       2 // is moving left or right from the box
//#define BOX_SIDE      3 // is following to the left or right of box
//#define BOX_REVERSE   4 // is moving opposite of DECIDED
//#define AT_TURN       5 // has found a right turn

// Direction Definitions
#define LEFT          true
#define RIGHT         false

// Turn Conditions
#define ON_LINE       0 // bot is on line or completely off
#define AT_T          1 // bot is at a T-intersection
#define AT_LEFT       2 // bot is at left turn
#define AT_RIGHT      3 // bot is at right turn

// Movement Actions
#define STOP          0 // bot completely stops
#define TURN_LEFT     1 // bot turns ~90 degrees left
#define TURN_RIGHT    2 // bot turns ~90 degrees right
#define MOVE_FORWARD  3 // bot moves forward at FULL_SPEED

#define MAX_VELOCITY  255  // maximum motor velocity

#define FULL_SPEED    0.75 // fraction of MAX_VELOCITY that is 'full' speed 
#define TURN_SPEED    0.90 // fraction of MAX_VELOCITY that is 'turning' speed 

// maximum difference in wheel speeds when line-following
#define MAX_PID_DELTA (1-FULL_SPEED)*MAX_VELOCITY  

// Course Locations
boolean leftRightLoc = RIGHT;  // (RIGHT or LEFT)
short   taskLoc      = -1;     // (-1 to 2)
short   mainLoc      = 0;      // (0 to 7)

// PID Coeffs
double KP = 1;
double KI = 0.05;
double KD = 0.25;

// Movement speeds
double leftDelta = 0;
double rightDelta = 0;
double forwardSpeed = 0;

// Change these pins when you need to
unsigned char fSensorPins[] = {31,33,35,37,39,41,43,45};
unsigned char rSensorPins[] = {2,3,4,5,6,7,8,9};
unsigned robotState = 0;

// Sensors (f)(c)0 through (f)(c)7 are connected to (f)(c)SensorPins[]
PololuQTRSensorsRC fSensor(fSensorPins, NUM_SENSORS, TIMEOUT); 
PololuQTRSensorsRC rSensor(rSensorPins, NUM_SENSORS, TIMEOUT); 
unsigned int fSensorValues[NUM_SENSORS];
unsigned int rSensorValues[NUM_SENSORS];

// Setup PID computation
double setpointPID, inputPID, outputPID;
PID lfPID(&inputPID, &outputPID, &setpointPID, KP, KI, KD, DIRECT);

void setup()
{
  setpointPID = MID_LINE;    // Set the point we want our PID loop to adjust to
  Serial.begin(9600);        // Begin serial comm for debugging  
  delay(500);                // Wait for serial comm to come online
  
  lfPID.SetMode(AUTOMATIC);  // turn on the PID
  lfPID.SetOutputLimits(-MAX_PID_DELTA, MAX_PID_DELTA); // force PID to the range of motor speeds. 
  
  // Calibrate sensors  (robot must be fully on the line)
  // Note: still needs calibration motor routine
  for (int i = 0; i < 200; i++)  // Make the calibration take about 5 seconds
  {
    // Reads both sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
    fSensor.calibrate();       
    rSensor.calibrate();
  }
  
  // Start movement (currently starting at mainLoc 'M0')
  mainLoc = 0;
  moveBot(MOVE_FORWARD);
}

void loop()
{ 
  switch (mainLoc)
  {
    case 0: // At mainLoc M0 -----------------------------------------------------
      followLine(); // Follow the line until turn is detected
      if (isTurn() == AT_RIGHT) {turnRight();} // Should only detect a right turn
      mainLoc = 1; // Bot is now at mainLoc M1
      moveBot(MOVE_FORWARD); //Begin moving forward again
      break;
    case 1: // At mainLoc M1 -----------------------------------------------------
      switch (taskLoc)
      {
        case -1:
          followLine(); // Follow the line until 'T' is detected
          if (isTurn() == AT_T)
          {
            moveToSensor();
            takeReading();
            moveFromSensor();
            if (leftRightLoc == LEFT) {turnLeft();}  
            else {turnRight();} 
            taskLoc = 0; // Bot is now at taskLoc L/R0
            moveBot(MOVE_FORWARD);
          }
          break;  
        case 0:
          followLine(); // Follow the line until turn is detected
          if (isTurn() == AT_LEFT) // If left turn detected
          {
            turnLeft();   // Face the M2 line again
            taskLoc = 1;  // Bot is now at taskLoc R1
            moveBot(MOVE_FORWARD); // Start moving forward
          } 
          else if (isTurn() == AT_RIGHT) // If left turn detected
          {
            turnRight();   // Face the M2 line again
            taskLoc = 1;   // Bot is now at taskLoc L1
            moveBot(MOVE_FORWARD); // Start moving forward
          }
          break;
        case 1:
          followLine(); // Follow the line until turn is detected
          if (isTurn() == AT_LEFT) // If left turn detected
          {
            moveOffLine(); // Move off of the line a bit
            turnLeft();    // Face the M2 line again
            taskLoc = 2;   // Bot is now at taskLoc R2
            moveBot(MOVE_FORWARD); // Start moving towards M2
          } 
          else if (isTurn() == AT_RIGHT) // If left turn detected
          {
            moveOffLine(); // Move off of the line a bit
            turnRight();   // Face the M2 line again
            taskLoc = 2;   // Bot is now at taskLoc L2
            moveBot(MOVE_FORWARD); // Start moving towards M2
          } 
          break;
        case 2:
          if (isTurn() == AT_T)
          {
            if (leftRightLoc == LEFT) {turnLeft();}  
            else {turnRight();}
            taskLoc = -1;  // Bot is now at taskLoc L/R-1
            mainLoc = 2;   // Bot is now at mainLoc M2
            moveBot
          }
          break;
      }
      break;
    case 2: // At mainLoc M2 -----------------------------------------------------
      // TODO
      break;
    case 3: // At mainLoc M3 -----------------------------------------------------
      followLine(); // Follow the line unless turn is detected
      if (isTurn() == AT_RIGHT) {turnRight();} // Should only detect a right turn
      mainLoc = 4; // Bot is now at mainLoc M4
      moveBot(MOVE_FORWARD); //Begin moving forward again
      break;
    case 4: // At mainLoc M4 -----------------------------------------------------
      followLine(); // Follow the line unless turn is detected
      if (isTurn() == AT_RIGHT) {turnRight();} // Should only detect a right turn
      mainLoc = 5; // Bot is now at mainLoc M5
      moveBot(MOVE_FORWARD); //Begin moving forward again
      break;
    case 5: // At mainLoc M5 -----------------------------------------------------
      // TODO
      break;
    case 6: // At mainLoc M6 -----------------------------------------------------
      break;
    case 7: // At mainLoc M7 -----------------------------------------------------
      followLine(); // Follow the line unless turn is detected
      if (isTurn() == AT_RIGHT) {turnRight();} // Should only detect a right turn
      mainLoc = 0; // Bot is now at mainLoc M0
      moveBot(MOVE_FORWARD); //Begin moving forward again
      break;
  }
  
  
}

// Checks to see if the robot is at a turn or a 'T', by checking the outer sensors.
// NOTE: This assumes white line on black surface.
int isTurn()
{
  boolean isLeft  = (fSensorValues[0] < REFLECT_THRESHOLD);
  boolean isRight = (fSensorValues[7] < REFLECT_THRESHOLD);
  if (isLeft && isRight) {return AT_T;}
  else if (isLeft) {return AT_LEFT;}
  else if (isRight) {return AT_RIGHT;}
  else {return ON_LINE;}
}

// Does PID for line folliwing and sets the motor delta speeds.
void followLine()
{
  // Read calibrated front sensor values and obtain a measure of the line position from 0 to 7000
  unsigned int position = fSensor.readLine(fSensorValues, QTR_EMITTERS_ON, WHITE_LINE);                            

    inputPID = position;            // set PID input to position of line
    lfPID.Compute();                // compute correction, store in outputPID
    rightDelta = outputPID;         // sets right wheel's speed variation
    leftDelta  = -outputPID;        // sets left wheel's speed variation
    
    Serial.print("Correction: ");   //for PID debugging
    Serial.println(outputPID);      //for PID debugging
}

// Bot move actions
void moveBot(int moveType)
{
  switch (moveType)
  {
    case STOP:
      forwardSpeed = 0;
      leftDelta    = 0;
      rightDelta   = 0;
      break;
    case TURN_LEFT:
      forwardSpeed = 0;
      leftDelta    = -TURN_SPEED * MAX_VELOCITY;
      rightDelta   = TURN_SPEED * MAX_VELOCITY;
      break;
    case TURN_RIGHT:
      forwardSpeed = 0;
      leftDelta    = TURN_SPEED * MAX_VELOCITY;
      rightDelta   = -TURN_SPEED * MAX_VELOCITY;
      break;
    case MOVE_FORWARD:
      forwardSpeed = FULL_SPEED * MAX_VELOCITY;
      leftDelta    = 0;
      rightDelta   = 0;
      break;
    default:
      forwardSpeed = FULL_SPEED * MAX_VELOCITY;
      leftDelta    = 0;
      rightDelta   = 0;
  }
}

// Turn bot 90 degrees to the left
void turnLeft()
{
  //TODO
}

// Turn bot 90 degrees to the right
void turnRight()
{
  //TODO
}

//Moves the bot up to the task sensors for a reading
void moveToSensor()
{
  //TODO
}

//Take a reading from the task sensors and makes L/R decision 
void takeReading()
{
  //TODO
}

//Moves the bot back from the task to the 'T'
void moveFromSensor()
{
  //TODO
}

//Moves the bot off of the line by going forward a bit
void moveOffLine()
{
  //TODO
}