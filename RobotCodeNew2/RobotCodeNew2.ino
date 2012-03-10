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


#define FTA_CYCLE_SIZE  21 // Defines the size (in bytes) of the ftaCycle struct.
#define NUM_SEGMENTS  40 //The number of segments on the course
#define NUM_SENSORS   16    // number of sensors used on each array
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define MID_LINE      ((NUM_SENSORS-1)*1000)/2  // value of sensor when line is centered (0-7000)
#define WHITE_LINE    1     // '1' = white line, '0' = black line
#define REFLECT_THRESHOLD 650  // part of 1000 at which line is not found

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
#define MOVE_FAST       9 // Bot moves at FULLSPEED
#define STOP_LEFT     10
#define STOP_RIGHT     11

#define MAX_VELOCITY  255  // Maximum motor velocity

// Movement speeds
double leftDelta = 0;
double rightDelta = 0;
double forwardSpeed = 0;

float FULL_SPEED = 0.4; // Fraction of MAX_VELOCITY that is 'full' speed, set each segment
float TURN_SPEED = 0.3; // Fraction of MAX_VELOCITY that is 'turning' speed, set each segment


// PWM offset for motor speeds to be equal (Left motor is faster = +)
#define MOTOR_OFFSET  0

PololuQTRSensorsRC fSensors(fSensorPins, NUM_SENSORS, TIMEOUT); 
unsigned int fSensorValues[NUM_SENSORS];

// Setup PID computation
double setpointPID, inputPID, outputPID;
PID lfPID(&inputPID, &outputPID, &setpointPID, 0.1,0.1,0.1, DIRECT);


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
  lfPID.SetOutputLimits(MAX_VELOCITY, MAX_VELOCITY); // force PID to the range of motor speeds.   
  lfPID.SetMode(AUTOMATIC); 
}


void loop(void)
{
/*    for(int i=0;i<NUM_SENSORS;i++)
    {
        Serial.print("Sensor[");
        Serial.print(i);
        Serial.print("]: ");
        Serial.print(fSensorValues[i]);
        Serial.print("\n");
        delay(100);
    }     */
    followLine();
}

void calibrateSensors()
{
  setMove(TURN_LEFT);
  // Calibrate sensors  (robot must be fully on the line)
  // Note: still needs calibration motor routine
  for (int i = 0; i < 75; i++)  // Make the calibration take about 5 seconds
  {
    // Reads both sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
    fSensors.calibrate(QTR_EMITTERS_ON);
  }

  setMove(TURN_RIGHT);
  for (int i = 0; i < 75; i++)  // Make the calibration take about 5 seconds
  {
    // Reads both sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
    fSensors.calibrate(QTR_EMITTERS_ON);
  }
  setMove(STOP);
}



