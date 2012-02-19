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
    case PIVOT_LEFT_FORWARD:
      forwardSpeed = 0;
      leftDelta    = 0;
      rightDelta   = TURN_SPEED * MAX_VELOCITY;
      break;
    case PIVOT_LEFT_BACK:
      forwardSpeed = 0;
      leftDelta    = 0;
      rightDelta   = -TURN_SPEED * MAX_VELOCITY;
      break;
    case PIVOT_RIGHT_FORWARD:
      forwardSpeed = 0;
      leftDelta    = TURN_SPEED * MAX_VELOCITY;
      rightDelta   = 0;
      break;
    case PIVOT_RIGHT_BACK:
      forwardSpeed = 0;
      leftDelta    = -TURN_SPEED * MAX_VELOCITY;
      rightDelta   = 0;
      break;
    case TURN_RIGHT:
      forwardSpeed = 0;
      leftDelta    = TURN_SPEED * MAX_VELOCITY;
      rightDelta   = -TURN_SPEED * MAX_VELOCITY;
      break;
    case MOVE_FORWARD:
      forwardSpeed = FULL_SPEED*MAX_VELOCITY;
      leftDelta    = 0;
      rightDelta   = 0;
      break;
    case MOVE_BACKWARD:
      forwardSpeed = -FULL_SPEED*MAX_VELOCITY;
      leftDelta    = 0;
      rightDelta   = 0;
      break;
    default:
      forwardSpeed = FULL_SPEED*MAX_VELOCITY;
      leftDelta    = 0;
      rightDelta   = 0;
  }
  updateMotors();
}

// Does PID for line folliwing and sets the motor delta speeds.
void followLine()
{
  // Read calibrated front sensor values and obtain a measure of the line position from 0 to 7000
  unsigned int pOsitIon = getLine();

  inputPID = pOsitIon;            // set PID input to position of line
  
//  Serial.print("S: ");
//  Serial.print(pOsitIon);
//  Serial.print("\n");
  lfPID.Compute();                // compute correction, store in outputPID
  if (outputPID < 0)
    rightDelta = outputPID;         // sets right wheel's speed variation
  else
    leftDelta  = -outputPID;        // sets left wheel's speed variation
  updateMotors();
  
  /*  Serial.print("\n");
        Serial.print("\n");
 Serial.print("LS[0]: ");
    Serial.print(fSensorValuesBoth[0]);
    Serial.print("\tLS[1]: ");
    Serial.print(fSensorValuesBoth[1]);
    Serial.print("\tLS[2]: ");
    Serial.print(fSensorValuesBoth[2]);
    Serial.print("\tLS[3]: ");
    Serial.print(fSensorValuesBoth[3]);
    Serial.print("\tLS[4]: ");
    Serial.print(fSensorValuesBoth[4]);
    Serial.print("\tLS[5]: ");
    Serial.print(fSensorValuesBoth[5]);
    Serial.print("\tLS[6]: ");
    Serial.print(fSensorValuesBoth[6]);
    Serial.print("\tLS[7]: ");
    Serial.println(fSensorValuesBoth[7]); 
    Serial.print("RS[0]: ");
    Serial.print(fSensorValuesBoth[8]);
    Serial.print("\tRS[1]: ");
    Serial.print(fSensorValuesBoth[9]);
    Serial.print("\tRS[2]: ");
    Serial.print(fSensorValuesBoth[10]);
    Serial.print("\tRS[3]: ");
    Serial.print(fSensorValuesBoth[11]);
    Serial.print("\tRS[4]: ");
    Serial.print(fSensorValuesBoth[12]);
    Serial.print("\tRS[5]: ");
    Serial.print(fSensorValuesBoth[13]);
    Serial.print("\tRS[6]: ");
    Serial.print(fSensorValuesBoth[14]);
    Serial.print("\tRS[7]: ");
    Serial.println(fSensorValuesBoth[15]);   
    Serial.print("\n");
        Serial.print("\n");*/
  #ifdef DEBUG_COURSE
    Serial.print("Counter: ");
    Serial.print(delayCounter);
    Serial.print("\tMainLoc: M");
    Serial.print(mainLoc);
    Serial.print("\tTaskLoc: T");
    Serial.println(taskLoc);
  #endif
}


//Move up to and back from the boxes with the encoders??
void encoderMoveToTerminate(int desiredTerminationType)
{
 // lEncoder.readCalibrated(lEncoderValues, QTR_EMITTERS_ON); 
 // rEncoder.readCalibrated(rEncoderValues, QTR_EMITTERS_ON); 
  int lCount = 5; 
  int rCount = 5;
  boolean isDone = false;
  boolean lLastColor = lEncoderValues[0] > 500;
  boolean rLastColor = rEncoderValues[0] > 500;
  int currentTerminationType;
  
  delay(50);
  
  if((location==2)||(location==11)||(location==20)||(location==29)){setMove(MOVE_FORWARD);}
  else if((location==3)||(location==12)||(location==21)||(location==30)){setMove(MOVE_BACKWARD);}
  
  Serial.print("here we go!");
  while(!isDone)
  {
      
    lEncoder.readCalibrated(lEncoderValues, QTR_EMITTERS_ON); 
    rEncoder.readCalibrated(rEncoderValues, QTR_EMITTERS_ON);
      currentTerminationType = checkTermination();
     if(lCount>=0||rCount>=0)
     {
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
      if(desiredTerminationType==OFF_LINE)
      {
      /*  Serial.print("OFF_ROAD!!");
        Serial.print("\n");
        Serial.print(rCount);
        Serial.print(" ");
        Serial.print(lCount);*/
        if(lCount<=0||rCount<=0)
        {
          isDone=true;
          break;
        }
      }
      else if(desiredTerminationType==ON_LINE)
      {
        if((currentTerminationType==ON_LINE)||(currentTerminationType>=AT_ANY))
        {
          if(lCount<=0||rCount<=0)
          {
            isDone=true;
            break;
          }
        }
      }
  }
  setMove(STOP);
}






/*  while(isDone == false)
  {
    while(rCount > 0 || lCount >  0)
    {
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
}*/

// Turn one wheel then the other by number of encoder clicks. 
// Notes: Positive clicks is forward. If rightFirst is true, turn right wheel first.
void turnLeftAndRight(int leftClicks, int rightClicks, boolean rightFirst)
{
  int lCount = leftClicks;  // Set the left countdown
  int rCount = rightClicks; // Set the right countdown
  boolean leftDone, rightDone;
  
  if (lCount < 0)
    lCount *= -1;
  if (rCount < 0)
    rCount *= -1;
  
  if (rightFirst) // Skip left and go to right
  {
    leftDone = true; 
    rightDone = false; 
  }
  else // Don't skip left; run in code order
  {
    leftDone = false;
    rightDone = false;
  }
  
  for (int i=0; i<2; ++i)
  {
    if (!leftDone)
    {
      // --- Turning left wheel: ---
      lEncoder.readCalibrated(lEncoderValues, QTR_EMITTERS_ON); // Read left encoder sensor
      
      if (leftClicks > 0)
        setMove(PIVOT_RIGHT_FORWARD);
      else if (leftClicks < 0) 
        setMove(PIVOT_RIGHT_BACK);
    
      boolean lLastColor = lEncoderValues[0] > 500;
     
      while(lCount > 0)
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
      leftDone = true;
    }
    
    if (!rightDone)
    {
      // --- Turning right wheel: ---
      rEncoder.readCalibrated(rEncoderValues, QTR_EMITTERS_ON); // Read left encoder sensor
      
      if (rightClicks > 0)
        setMove(PIVOT_LEFT_FORWARD);
      else if (rightClicks < 0) 
        setMove(PIVOT_LEFT_BACK);
    
      boolean rLastColor = rEncoderValues[0] > 500;
     
      while(rCount > 0)
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
      rightDone = true;
      leftDone = false;
      if (!rightFirst)
        break; // Break from the for loop, so the turns don't happen twice.
    }
  }
    
  setMove(STOP);
  Serial.print("STOP");
}

// Turn in place by number of encoder clicks. (Turn right is positive)
void turnInPlace(int clicks)
{
  int lCount, rCount = clicks; // Set the left and right countdowns
    
  lEncoder.readCalibrated(lEncoderValues, QTR_EMITTERS_ON); // Read left encoder sensor
  rEncoder.readCalibrated(rEncoderValues, QTR_EMITTERS_ON); // Read right encoder sensor
  if (clicks > 0) // Clicks is positive so rotate right
    setMove(TURN_RIGHT);
  else if (clicks < 0){ // Clicks is negative so rotate left
    setMove(TURN_LEFT);clicks*=-1;lCount;rCount = clicks;}

  boolean lLastColor = lEncoderValues[0] > 500; // Find initial left color value
  boolean rLastColor = rEncoderValues[0] > 500; // Find initial right color value
  
  while(rCount > 0 || lCount >  0)
  {
    if (lCount == 0) // If left finishes first, stop turning left wheel
    {
      leftDelta = 0;
      updateMotors();
    }
    if (rCount == 0) // If right finishes first, stop turning right wheel
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
}


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

unsigned int getLine()
{
  unsigned char i, on_line = 0;
  unsigned long avg; // this is for the weighted total, which is long
	                   // before division
  unsigned int sum; // this is for the denominator which is <= 64000
  static int last_value=0; // assume initially that the line is left.
  fSensorRight.readCalibrated(fSensorValuesRight, QTR_EMITTERS_ON);
  fSensorLeft.readCalibrated(fSensorValuesLeft, QTR_EMITTERS_ON);
  for(int i=0;i<8;i++)
  {
    fSensorValuesBoth[i]=fSensorValuesLeft[i];
  }
  for(int i=8;i<16;i++)
  {
    fSensorValuesBoth[i]=fSensorValuesRight[i-8];
  }

  avg = 0;
  sum = 0;
  
  for(i=0; i<16; i++) {
    int value = fSensorValuesBoth[i];
    value = 1000-value;
    // keep track of whether we see the line at all
    if(value > 200) {
      on_line = 1;
    }
      
    // only average in values that are above a noise threshold
    if(value > 50) {
      avg += (long)(value) * (i * 1000);
      sum += value;
    }
  }
    
  if(!on_line)
  {
    // If it last read to the left of center, return 0.
    if(last_value < 3500)
    return 0;
    
    // If it last read to the right of center, return the max.
    else
    return 7000;
    
  }
  
  last_value = (avg/sum)/2; //0-7000
  
  return last_value;
}
