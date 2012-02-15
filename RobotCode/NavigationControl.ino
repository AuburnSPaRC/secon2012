#include <EEPROM.h>

#define FTA_CYCLE_SIZE  7 // Defines the size (in bytes) of the ftaCycle struct.

// Follow types:
#define LINE_FOLLOW    0
#define ENCODER_TRAVEL 1

// Termination types:
#define ON_LINE       0 // Bot is on the line
#define OFF_LINE      1 // Bot is off the line
#define HIT_SWITCH    2 // Physical switch is triggered
#define AT_ANY        3 // Bot is at any type of turn (all types above this line do not count)
#define AT_T          4 // Bot is at a T-intersection
#define AT_LEFT       5 // Bot is at left turn
#define AT_RIGHT      6 // Bot is at right turn






// Executes the three-stage cycle (follow, terminate, action) for a given segment of course.
void executeSegment(int segment)
{
  // --- Retrieve FTA information from EEPROM: ---
  ftaCycle currentSegment;
  currentSegment.follow = EEPROM.read(segment * FTA_CYCLE_SIZE);          // Read the follow type
  currentSegment.terminate = EEPROM.read((segment * FTA_CYCLE_SIZE) + 1); // Read the termination type
  currentSegment.action = EEPROM.read((segment * FTA_CYCLE_SIZE) + 2);    // Read the action type
  byte tempMSB = EEPROM.read((segment * FTA_CYCLE_SIZE) + 3); // Read MSB of leftAmount
  byte tempLSB = EEPROM.read((segment * FTA_CYCLE_SIZE) + 4); // Read LSB of leftAmount
  currentSegment.leftAmount = word(tempMSB,tempLSB);     // Combine MSB and LSB
  tempMSB = EEPROM.read((segment * FTA_CYCLE_SIZE) + 5); // Read MSB of rightAmount
  tempLSB = EEPROM.read((segment * FTA_CYCLE_SIZE) + 6); // Read LSB of rightAmount
  currentSegment.rightAmount = word(tempMSB,tempLSB);    // Combine MSB and LSB

  // --- Execute line- or encoder-following: ---
  if (currentSegment.follow == LINE_FOLLOW) // Line-following type movement
  {
    // Execute linefollowing until termination occurs:
    lfPID.SetMode(AUTOMATIC); // Turn on PID
    int terminationType = checkTermination();
    if (currentSegment.terminate == AT_ANY) // Special case for AT_ANY
    {
      while (terminationType < AT_ANY)
      {
        followLine();
        terminationType = checkTermination();
      }
    }
    else // All other termination types
    {
      while (terminationType != currentSegment.terminate)
      {
        setMove(MOVE_FORWARD); // Begin moving forward
        followLine();
        terminationType = checkTermination();
      }
    }
    setMove(STOP);
    lfPID.SetMode(MANUAL); // Turn off PID
  }
  else  // Encoder-travel type movement
  {
    // Execute encoder-following until termination occurs:
    encoderMoveToTerminate(currentSegment.terminate);
  }

  // --- Perform appropriate action: ---
  // TODO:
  
}

// Checks to see if the robot is at a turn or a 'T', by checking the outer sensors.
// NOTE: This assumes white line on black surface.
int checkTermination()
{
  boolean isLeft  = (fSensorValues[0] < REFLECT_THRESHOLD);
  boolean isRight = (fSensorValues[7] < REFLECT_THRESHOLD);
  boolean isOff = true;
  
  // Checks to see if every sensor is above threshold:
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    isOff &= (fSensorValues[i] >= REFLECT_THRESHOLD);
  }
  
  if (isLeft && isRight) 
    return AT_T;
  else if (isLeft) 
    return AT_LEFT;
  else if (isRight) 
    return AT_RIGHT;
  else if (isOff)
    return OFF_LINE;
  else 
    return ON_LINE;
}

// Old navigation loop for temporary reference.
// NOTE: DO NOT USE THIS FUNCTION!
//void oldLoop()
//{
//  
//  int turnType = isTurn();
//  //dynamic_PID();
//  switch (mainLoc)
//  {
//    case 0: // At mainLoc M0
//      followLine(); // Follow the line until turn is detected
//      if (turnType == AT_RIGHT || turnType == OFF_LINE) 
//      {
//        turnRight(); // Should only detect a right turn
//        mainLoc = 1; // Bot is now at mainLoc M1
//        setMove(MOVE_FORWARD); // Begin moving forward again
//      }
//      break;
//    case 1: // At mainLoc M1
//      navigateTask();
//      break;
//    case 2: // At mainLoc M2
//      navigateTask();
//      break;
//    case 3: // At mainLoc M3
//      followLine(); // Follow the line unless turn is detected
//      if (turnType == AT_RIGHT || turnType == OFF_LINE)
//      {
//        turnRight(); // Should only detect a right turn
//        mainLoc = 4; // Bot is now at mainLoc M4
//        setMove(MOVE_FORWARD); // Begin moving forward again
//      }
//      break;
//    case 4: // At mainLoc M4
//      followLine(); // Follow the line unless turn is detected
//      if (turnType == AT_RIGHT || turnType == OFF_LINE)
//      {
//        turnRight(); // Should only detect a right turn
//        mainLoc = 5; // Bot is now at mainLoc M5
//        setMove(MOVE_FORWARD); // Begin moving forward again
//
//      }
//      break;
//    case 5: // At mainLoc M5
//      navigateTask();
//      break;
//    case 6: // At mainLoc M6
//      navigateTask();
//      break;
//    case 7: // At mainLoc M7
//      followLine(); // Follow the line unless turn is detected
//      if (turnType == AT_RIGHT || turnType == OFF_LINE) 
//      {
//        turnRight(); // Should only detect a right turn
//        mainLoc = 0; // Bot is now at mainLoc M0
//        setMove(MOVE_FORWARD); // Begin moving forward again
//      }
//      break;
//  }  
//}
//
//// Old navigation loop for temporary reference.
//// NOTE: DO NOT USE THIS FUNCTION!
//// Navigate around the task and take the sensor measurement
//void navigateTask()
//{
//  int turnType = isTurn();
//  switch (taskLoc)
//  {
//    case -1:
//      followLine(); // Follow the line until 'T' is detected
//      if (turnType == AT_T || turnType == OFF_LINE)
//      {
//        moveToSensor();
//        takeReading();
//        moveFromSensor();
//        if (leftRightLoc == LEFT)
//        { 
//          turnLeftWheel(-6);
//          turnRightWheel(32);
//        }
//        else 
//        {
//          turnRightWheel(-6);
//          turnLeftWheel(32);
//        }
//        taskLoc = 0; // Bot is now at taskLoc L/R0
//        setMove(MOVE_FORWARD);
//      }
//      break;  
//    case 0:
//      moveToTurn(); // Follow the line until turn is detected 
//      if (leftRightLoc == RIGHT)// && (turnType == AT_LEFT))// || turnType == OFF_LINE)) // If left turn detected
//      {
//        turnLeft(); 
//        taskLoc = 1;  // Bot is now at taskLoc R1
//        setMove(MOVE_FORWARD); // Start moving forward
//        delayCounter = 0; // Reset the delay counter
//      } 
//      else if (leftRightLoc == LEFT)// && (turnType == AT_RIGHT))// || turnType == OFF_LINE)) // If left turn detected
//      {
//        turnRight(); 
//        taskLoc = 1;   // Bot is now at taskLoc L1
//        setMove(MOVE_FORWARD); // Start moving forward
//        delayCounter = 0; // Reset the delay counter
//      }
//    
//      break;
//    case 1:
//      followLine(); // Follow the line until turn is detected
//      if (leftRightLoc == RIGHT && (turnType == AT_LEFT))// || turnType == OFF_LINE)) // If left turn detected
//      {
//        turnLeft();    
//        taskLoc = 2;   // Bot is now at taskLoc R2
//        setMove(MOVE_FORWARD); // Start moving forward
//      } 
//      else if (leftRightLoc == LEFT && (turnType == AT_RIGHT))// || turnType == OFF_LINE)) // If left turn detected
//      {
//        turnRight();  
//        taskLoc = 2;   // Bot is now at taskLoc L2
//        setMove(MOVE_FORWARD); // Start moving forward
//        delayCounter = 0; // Reset the delay counter
//      } 
//      
//      break;
//    case 2:
//      followLine(); // Follow the line until turn is detected
//      if (leftRightLoc == RIGHT && (turnType == AT_RIGHT))// || turnType == OFF_LINE)) // If left turn detected
//      {
//        turnRight();    
//        taskLoc = -1;      // Reset taskLoc
//        increaseMainLoc(); // Increments mainLoc by 1
//        setMove(MOVE_FORWARD); // Start moving forward
//        delayCounter = 0; // Reset the delay counter
//      } 
//      else if (leftRightLoc == LEFT && (turnType == AT_LEFT))// || turnType == OFF_LINE)) // If left turn detected
//      {
//        turnLeft();       
//        taskLoc = -1;      // Reset taskLoc
//        increaseMainLoc(); // Increments mainLoc by 1
//        setMove(MOVE_FORWARD); // Start moving forward
//        delayCounter = 0; // Reset the delay counter
//      } 
//      break;
//  } 
//}
