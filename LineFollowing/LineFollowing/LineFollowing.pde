#include <PID_v1.h>
#include <PololuQTRSensors.h>

#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2
#define MUX_S0_PIN    11    // mux select pin LSB
#define MUX_S1_PIN    12    // mux select pin MSB
#define MID_LINE      3500  // value of sensor when line is centered

// Robot States
#define ON_MAIN       0 // following a the main line
#define AT_BOX        1 // leaves from straight line to 'T'
#define DECIDED       2 // is moving left or right from the box
#define FOUND_SIDE    3 // has found to the line next to box
#define BOX_SIDE      4 // is following to the left or right of box
#define BOX_TURN      5 // is moving opposite of DECIDED
#define FOUND_MAIN    6 // found the main line again
#define AT_TURN       7 // has found a right turn

// PID Coeffs
#define KP            2
#define KI            0 
#define KD            1

#define MIN_VELOCITY  -255 // minimum motor velocity
#define MAX_VELOCITY  255  // maximum motor velocity

int* leftVelocity = NULL;
int* rightVelocity = NULL;
int* forwardVelocity = NULL;
int* backwardVelocity  = NULL;
unsigned* leftSensor = NULL;
unsigned* rightSensor = NULL;
unsigned* frontSensor = NULL;
unsigned* rearSensor = NULL;

unsigned char sensorPins[] = {3, 4, 5, 6, 7, 8, 9, 10};
unsigned robotState = 0;

// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
PololuQTRSensorsRC qtrrc(sensorPins, NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];

double setpointPID, inputPID, outputPID;
PID lfPID(&inputPID, &outputPID, &setpointPID, KP, KI, KD, DIRECT);

void setup()
{
  Serial.begin(9600);
  delay(500);
  
  *leftVelocity = 0;
  *rightVelocity = 0;
  *forwardVelocity = 0;
  *backwardVelocity = 0;
  *leftSensor = 4;
  *rightSensor = 2;
  *frontSensor = 1;
  *rearSensor = 3;
  
  lfPID.SetMode(AUTOMATIC);  // turn on the PID
  lfPID.SetOutputLimits(MIN_VELOCITY, MAX_VELOCITY); // force PID to range of motor speeds. 
  
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }

}


void loop()
{
  if (ON_MAIN)
  {
    
    
    // Read calibrated sensor values and obtain a measure of the line position from 0 to 7000
    unsigned int position = qtrrc.readLine(sensorValues, QTR_EMITTERS_ON, 1); //for white line
    if (!(position > 0 && position < 7000))
    {
      if(isTurn()) {robotState = AT_TURN;}
      else {robotState = AT_BOX;}
    }
    else
    {
      inputPID = position;            // set PID input to position of line
      lfPID.Compute();                // compute correction, store in outputPID
      if (outputPID > 0) {*rightVelocity = outputPID;} // positive correction means move right
      else {*leftVelocity = -outputPID;}               // negative correction means move left
      
      Serial.print("Correction: ");  
      Serial.println(outputPID);
      
    }
  }
}

// Checks to see if the robot is at a turn or just at the 'T'.
boolean isTurn()
{
  return false;
}

// Rotates where the 'front' of the robot is in relation to the sensors and motors. Positive value of turns
//  means rotate 'right' which means what used to be 'right' is now 'forward'.
void rotateAxes(unsigned turns)
{
  int* tempInt;
  unsigned* tempUns;
  for (int i = turns; i > 0; i--)
  {
    leftVelocity = backwardVelocity;
    rightVelocity = forwardVelocity;
    forwardVelocity = leftVelocity;
    backwardVelocity = rightVelocity;
    leftSensor = rearSensor;
    rightSensor = frontSensor;
    frontSensor = leftSensor;
    rearSensor = rightSensor;
  }
  
}
