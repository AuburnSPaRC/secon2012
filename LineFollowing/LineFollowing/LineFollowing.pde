#include <PID_v1.h>
#include <PololuQTRSensors.h>

#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN   7    // emitter is controlled by digital pin 7
#define MUX_S0_PIN    11    // mux select pin LSB
#define MUX_S1_PIN    12    // mux select pin MSB
#define MID_LINE      3500  // value of sensor when line is centered

#define REFLECT_THRESHOLD 750  // fraction of 1000 at which line is not found

// Robot States
#define ON_MAIN       0 // following a the main line
#define AT_BOX        1 // leaves from straight line to 'T'
#define DECIDED       2 // is moving left or right from the box
#define FOUND_SIDE    3 // has found to the line next to box
#define BOX_SIDE      4 // is following to the left or right of box
#define BOX_TURN      5 // is moving opposite of DECIDED
#define FOUND_MAIN    6 // found the main line again
#define AT_TURN       7 // has found a right turn

#define MIN_VELOCITY  -255 // minimum motor velocity
#define MAX_VELOCITY  255  // maximum motor velocity


// PID Coeffs
double KP=1;
double KI=0.05;
double KD=0.25;


int* leftVelocity = NULL;
int* rightVelocity = NULL;
int* forwardVelocity = NULL;
int* backwardVelocity  = NULL;
unsigned* leftSensor = NULL;
unsigned* rightSensor = NULL;
unsigned* frontSensor = NULL;
unsigned* rearSensor = NULL;

//Change these when you need to
unsigned char sensorPins[] = {10,6,8,13,5,4,3,2};
unsigned robotState = 0;

// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
PololuQTRSensorsRC qtrrc(sensorPins, NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];

double setpointPID, inputPID, outputPID;
PID lfPID(&inputPID, &outputPID, &setpointPID, KP, KI, KD, DIRECT);

void setup()
{
  setpointPID=MID_LINE;    //Set the point we want our PID loop to adjust to
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
  
  // Setup mux outputs
  pinMode(MUX_S0_PIN, OUTPUT);
  pinMode(MUX_S1_PIN, OUTPUT);
  
  lfPID.SetMode(AUTOMATIC);  // turn on the PID
  lfPID.SetOutputLimits(MIN_VELOCITY, MAX_VELOCITY); // force PID to range of motor speeds. 
  
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
  
  //This is to calibrate the sensors, we need to set the maximum range of the values or else find a way to have the robot move over the line back and forth once before it starts to get a 
  //range of values we're going to see.
  Serial.println();
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
    Serial.println();
    delay(1000);
}


void loop()
{ 
  if (robotState==ON_MAIN)
  {
    // Read calibrated front sensor values and obtain a measure of the line position from 0 to 7000
    activateSensor(*frontSensor);
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
  else if (robotState==AT_TURN)
  {  
  }
  else if (robotState==AT_BOX)
  {


    
  }
  
}

// Checks to see if the robot is at a turn or just at the 'T'.
boolean isTurn()
{
  boolean leftPresent = false;
  boolean rightPresent = false;
  
  activateSensor(*leftSensor);
  qtrrc.readCalibrated(sensorValues, QTR_EMITTERS_ON);
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    if (sensorValues[i] < REFLECT_THRESHOLD)
    {
      leftPresent = true;
      rotateAxes(4);
    }
  }
  
  activateSensor(*rightSensor);
  qtrrc.readCalibrated(sensorValues, QTR_EMITTERS_ON);
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    if (sensorValues[i] < REFLECT_THRESHOLD) 
    {
      rightPresent = true;
      rotateAxes(1);
    }
  }

  return leftPresent || rightPresent;
}

// Rotates where the 'front' of the robot is in relation to the sensors and motors. Positive value of turns
//  means rotate 'right' which means what used to be 'right' is now 'forward'.
void rotateAxes(unsigned turns)
{
  int* tempInt;
  unsigned* tempUns;
  for (int i = turns; i > 0; i--)
  {
    tempInt = leftVelocity;
    leftVelocity = backwardVelocity;
    backwardVelocity = rightVelocity;
    rightVelocity = forwardVelocity;
    forwardVelocity = tempInt;
    
    tempUns = leftSensor;
    leftSensor = rearSensor;
    rearSensor = rightSensor;
    rightSensor = frontSensor;
    frontSensor = tempUns;
  } 
}

// Activates a sensor based on its value (1-4)
void activateSensor(int sensor)
{
  switch (sensor) 
  {
    case 1:
      digitalWrite(MUX_S0_PIN, LOW);
      digitalWrite(MUX_S1_PIN, LOW);
    case 2:
      digitalWrite(MUX_S0_PIN, LOW);
      digitalWrite(MUX_S1_PIN, HIGH);
    case 3:
      digitalWrite(MUX_S0_PIN, HIGH);
      digitalWrite(MUX_S1_PIN, LOW);
    default:
      digitalWrite(MUX_S0_PIN, HIGH);
      digitalWrite(MUX_S1_PIN, HIGH);
  } 
}
