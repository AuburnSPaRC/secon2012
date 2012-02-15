/**  Code for the defining the motion of the robot.
* 
* Defines the motions and trajectories that the robot 
* may make on the course. Included motions are turning,
* forward/backward, approach task, leave task, etc.
*
* Depends: MotorFunctions.ino
*
* Credits: Tyler Crumpton
*
*/

// Termination types:
#define ON_LINE       0 // Bot is on the line
#define OFF_LINE      1 // Bot is off the line
#define HIT_SWITCH    2 // Physical switch is triggered
#define AT_ANY        3 // Bot is at any type of turn (all types above this line do not count)
#define AT_T          4 // Bot is at a T-intersection
#define AT_LEFT       5 // Bot is at left turn
#define AT_RIGHT      6 // Bot is at right turn

// Bot move actions
void setMove(int moveType)
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
  updateMotors();
}

// Does PID for line folliwing and sets the motor delta speeds.
void followLine()
{
  // Read calibrated front sensor values and obtain a measure of the line position from 0 to 7000
  unsigned int position = fSensor.readLine(fSensorValues, QTR_EMITTERS_ON, WHITE_LINE);                            

  inputPID = position;            // set PID input to position of line
  lfPID.Compute();                // compute correction, store in outputPID
  if (outputPID < 0)
    rightDelta = outputPID;         // sets right wheel's speed variation
  else
    leftDelta  = -outputPID;        // sets left wheel's speed variation
  updateMotors();
  
  #ifdef DEBUG_LINE
    Serial.print("Position: ");
    Serial.print(position);
    Serial.print("S[0]: ");
    Serial.print(fSensorValues[0]);
    Serial.print("\tS[1]: ");
    Serial.print(fSensorValues[1]);
    Serial.print("\tS[2]: ");
    Serial.print(fSensorValues[2]);
    Serial.print("\tS[3]: ");
    Serial.print(fSensorValues[3]);
    Serial.print("\tS[4]: ");
    Serial.print(fSensorValues[4]);
    Serial.print("\tS[5]: ");
    Serial.print(fSensorValues[5]);
    Serial.print("\tS[6]: ");
    Serial.print(fSensorValues[6]);
    Serial.print("\tS[7]: ");
    Serial.println(fSensorValues[7]);    
  #endif  
  #ifdef DEBUG_COURSE
    Serial.print("Counter: ");
    Serial.print(delayCounter);
    Serial.print("\tMainLoc: M");
    Serial.print(mainLoc);
    Serial.print("\tTaskLoc: T");
    Serial.println(taskLoc);
  #endif
}

void encoderMoveToTerminate(int desiredTerminationType)
{
  lEncoder.readCalibrated(lEncoderValues, QTR_EMITTERS_ON); 
  rEncoder.readCalibrated(rEncoderValues, QTR_EMITTERS_ON); 
  setMove(MOVE_FORWARD);
  int lCount = 5; 
  int rCount = 5;
  boolean isDone = false;
  boolean lLastColor = lEncoderValues[0] > 500;
  boolean rLastColor = rEncoderValues[0] > 500;
  int currentTerminationType;
  while(isDone == false)
  {
    while(rCount > 0 || lCount >  0)
    {
      fSensor.readCalibrated(fSensorValues, QTR_EMITTERS_ON);
      currentTerminationType = checkTermination();
      if (currentTerminationType > AT_ANY)
      {
        if (desiredTerminationType == AT_ANY || currentTerminationType == desiredTerminationType)
        {
          rCount = lCount = 0;
          isDone = true;
          break; 
        }  
      }
      if (lCount == 0)
      {
        leftDelta = 0;
        updateMotors();
      }
      if (rCount == 0)
      {
        rightDelta = 0;
        updateMotors();
      }
      
      lEncoder.readCalibrated(lEncoderValues, QTR_EMITTERS_ON); 
      rEncoder.readCalibrated(rEncoderValues, QTR_EMITTERS_ON); 
      
      if (lLastColor)
      {
        if (lEncoderValues[0] < 300)
        {
          lLastColor = !lLastColor;
          lCount--;
        }
      }
      else
      {
        if (lEncoderValues[0] > 700)
        {
          lLastColor = !lLastColor;
          lCount--;
        }
      }
      
      if (rLastColor)
      {
        if (rEncoderValues[0] < 300)
        {
          rLastColor = !rLastColor;
          rCount--;
        }
      }
      else
      {
        if (rEncoderValues[0] > 700)
        {
          rLastColor = !rLastColor;
          rCount--;
        }
      }   
    }
  }
  setMove(STOP);
}

// Turn bot 90 degrees to the left
void turnLeft()
{
  lfPID.SetMode(MANUAL);  // turn off the PID
  lEncoder.readCalibrated(lEncoderValues, QTR_EMITTERS_ON); 
  rEncoder.readCalibrated(rEncoderValues, QTR_EMITTERS_ON); 
  setMove(TURN_LEFT);
  //delay(TURN_TIME);
  int lCount = 19;
  int rCount = 19;
  boolean lLastColor = lEncoderValues[0] > 500;
  boolean rLastColor = rEncoderValues[0] > 500;
  
  
  while(rCount > 0 || lCount >  0)
  {
    if (lCount == 0)
    {
      leftDelta = 0;
      updateMotors();
    }
    if (rCount == 0)
    {
      rightDelta = 0;
      updateMotors();
    }
    
    lEncoder.readCalibrated(lEncoderValues, QTR_EMITTERS_ON); 
    rEncoder.readCalibrated(rEncoderValues, QTR_EMITTERS_ON); 
    
    if (lLastColor)
    {
      if (lEncoderValues[0] < 300)
      {
        lLastColor = !lLastColor;
        lCount--;
      }
    }
    else
    {
      if (lEncoderValues[0] > 700)
      {
        lLastColor = !lLastColor;
        lCount--;
      }
    }
    
    if (rLastColor)
    {
      if (rEncoderValues[0] < 300)
      {
        rLastColor = !rLastColor;
        rCount--;
      }
    }
    else
    {
      if (rEncoderValues[0] > 700)
      {
        rLastColor = !rLastColor;
        rCount--;
      }
    }
      
  }
  setMove(STOP);
  lfPID.SetMode(AUTOMATIC);  // turn on the PID
}

void turnLeftWheel(int stops)
{
  int lCount = stops;
  lfPID.SetMode(MANUAL);  // turn off the PID
  lEncoder.readCalibrated(lEncoderValues, QTR_EMITTERS_ON); 
  
  if (stops > 0)
  {
    leftDelta = TURN_SPEED * MAX_VELOCITY;
  }
  else
  {
    leftDelta = -TURN_SPEED * MAX_VELOCITY;
    lCount = -lCount;
  }
  
  updateMotors();
  
  boolean lLastColor = lEncoderValues[0] > 500;
  
  while(lCount >  0)
  {
    lEncoder.readCalibrated(lEncoderValues, QTR_EMITTERS_ON); 

    if (lLastColor)
    {
      if (lEncoderValues[0] < 300)
      {
        lLastColor = !lLastColor;
        lCount--;
      }
    }
    else
    {
      if (lEncoderValues[0] > 700)
      {
        lLastColor = !lLastColor;
        lCount--;
      }
    }      
  }
  setMove(STOP);
  lfPID.SetMode(AUTOMATIC);  // turn on the PID
}

// Turn bot 90 degrees to the right
void turnRight()
{
  lfPID.SetMode(MANUAL);  // turn off the PID
  lEncoder.readCalibrated(lEncoderValues, QTR_EMITTERS_ON); 
  rEncoder.readCalibrated(rEncoderValues, QTR_EMITTERS_ON); 
  setMove(TURN_RIGHT);
  //delay(TURN_TIME);
  int lCount = 19; 
  int rCount = 19;
  boolean lLastColor = lEncoderValues[0] > 500;
  boolean rLastColor = rEncoderValues[0] > 500;
  
  while(rCount > 0 || lCount >  0)
  {
    
    if (lCount == 0)
    {
      leftDelta = 0;
      updateMotors();
    }
    if (rCount == 0)
    {
      rightDelta = 0;
      updateMotors();
    }
    
    Serial.println(lCount);
    
    lEncoder.readCalibrated(lEncoderValues, QTR_EMITTERS_ON); 
    rEncoder.readCalibrated(rEncoderValues, QTR_EMITTERS_ON); 
    
    Serial.println(lCount);
    
    if (lLastColor)
    {
      if (lEncoderValues[0] < 300)
      {
        lLastColor = !lLastColor;
        lCount--;
      }
    }
    else
    {
      if (lEncoderValues[0] > 700)
      {
        lLastColor = !lLastColor;
        lCount--;
      }
    }
    
    if (rLastColor)
    {
      if (rEncoderValues[0] < 300)
      {
        rLastColor = !rLastColor;
        rCount--;
      }
    }
    else
    {
      if (rEncoderValues[0] > 700)
      {
        rLastColor = !rLastColor;
        rCount--;
      }
    }
      
  }
  setMove(STOP);
  lfPID.SetMode(AUTOMATIC);  // turn on the PID
}

void turnRightWheel(int stops)
{
  int rCount = stops;
  lfPID.SetMode(MANUAL);  // turn off the PID
  rEncoder.readCalibrated(rEncoderValues, QTR_EMITTERS_ON); 
  
  if (stops > 0)
    rightDelta = TURN_SPEED * MAX_VELOCITY;
  else
  {
    rightDelta = TURN_SPEED * MAX_VELOCITY;
    rCount = -rCount;
  }
  
  updateMotors();
  
  boolean rLastColor = rEncoderValues[0] > 500;
  
  while(rCount >  0)
  {
    rEncoder.readCalibrated(rEncoderValues, QTR_EMITTERS_ON); 

    if (rLastColor)
    {
      if (rEncoderValues[0] < 300)
      {
        rLastColor = !rLastColor;
        rCount--;
      }
    }
    else
    {
      if (rEncoderValues[0] > 700)
      {
        rLastColor = !rLastColor;
        rCount--;
      }
    }      
  }
  setMove(STOP);
  lfPID.SetMode(AUTOMATIC);  // turn on the PID
}

//void moveToTurn()
//{
//  lfPID.SetMode(MANUAL);  // turn off the PID
//  lEncoder.readCalibrated(lEncoderValues, QTR_EMITTERS_ON); 
//  rEncoder.readCalibrated(rEncoderValues, QTR_EMITTERS_ON); 
//  setMove(MOVE_FORWARD);
//  int lCount = 5; 
//  int rCount = 5;
//  int multiplier = 4;
//  boolean lLastColor = lEncoderValues[0] > 500;
//  boolean rLastColor = rEncoderValues[0] > 500;
//  while(multiplier > 0)
//  {
//    while(rCount > 0 || lCount >  0)
//    {
//      fSensor.readCalibrated(fSensorValues, QTR_EMITTERS_ON);
//      if (isTurn() == OFF_LINE)
//      {
//        rCount = lCount = multiplier = 0;
//        break; 
//      }
//      if (lCount == 0)
//      {
//        leftDelta = 0;
//        updateMotors();
//      }
//      if (rCount == 0)
//      {
//        rightDelta = 0;
//        updateMotors();
//      }
//      
//      lEncoder.readCalibrated(lEncoderValues, QTR_EMITTERS_ON); 
//      rEncoder.readCalibrated(rEncoderValues, QTR_EMITTERS_ON); 
//      
//      if (lLastColor)
//      {
//        if (lEncoderValues[0] < 300)
//        {
//          lLastColor = !lLastColor;
//          lCount--;
//        }
//      }
//      else
//      {
//        if (lEncoderValues[0] > 700)
//        {
//          lLastColor = !lLastColor;
//          lCount--;
//        }
//      }
//      
//      if (rLastColor)
//      {
//        if (rEncoderValues[0] < 300)
//        {
//          rLastColor = !rLastColor;
//          rCount--;
//        }
//      }
//      else
//      {
//        if (rEncoderValues[0] > 700)
//        {
//          rLastColor = !rLastColor;
//          rCount--;
//        }
//      }
//        
//    }
//    
//    multiplier--;
//  }
//  setMove(STOP);
//  lfPID.SetMode(AUTOMATIC);  // turn on the PID    
//}

////Moves the bot up to the task sensors for a reading
//void moveToSensor()
//{
//  //TODO: Add actual implementation
//  setMove(STOP);
//  delay(5000);
//}

////Moves the bot back from the task to the 'T'
//void moveFromSensor()
//{
//  //TODO
//}

// Spins the bot 5 times for speed measurement.
// NOTE: Requires #define CALIBRATE_MOTORS
void motorCalibrate()
{
  #ifdef CALIBRATE_MOTORS
    while(1)
    {
      setMove(TURN_RIGHT);
      delay(20 * TURN_TIME);
      setMove(STOP);
      delay(2000);
      setMove(TURN_LEFT);
      delay(20 * TURN_TIME);
      setMove(STOP);
      delay(2000);
    }  
  #endif
  #ifdef CALIBRATE_ENCODERS
    calibrateSensors();
    delay(5000);
    while(1)
    {
      turnLeftWheel(-6);
      //delay(2000);
      turnRightWheel(32);
      delay(5000);
    }
  #endif
}
