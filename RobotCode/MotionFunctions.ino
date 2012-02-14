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
