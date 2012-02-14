#include <EEPROM.h>

#define FTA_CYCLE_SIZE  7 // Defines the size (in bytes) of the ftaCycle struct.

// Follow types:
#define FOLLOW_LINE    0
#define ENCODER_TRAVEL 1

// Termination types:
#define ON_LINE       0 // Bot is on the line
#define AT_T          1 // Bot is at a T-intersection
#define AT_LEFT       2 // Bot is at left turn
#define AT_RIGHT      3 // Bot is at right turn
#define OFF_LINE      4 // Bot is off the line
#define HIT_SWITCH    5 // Physical switch is triggered

// Container for the "follow, terminate, action" cycle values.
struct ftaCycle{
  byte follow;      // "Line-following" vs. "simple encoder travel"
  byte terminate;   // "At right turn", "at left turn", "off the line", etc.
  byte action;      // "turn in place", "turn left wheel then right", "turn right wheel then left"
  int leftAmount;   // Number of encoder clicks to turn left wheel forward (negative is backward)
  int rightAmount;  // Number of encoder clicks to turn right wheel forward (negative is backward)
};

// Executes the three-stage cycle (follow, terminate, action) for a given segment of course.
void executeSegment(int segment)
{
  // Retrieve FTA information from EEPROM:
  ftaCycle current;
  current.follow = EEPROM.read(segment * FTA_CYCLE_SIZE);          // Read the follow type
  current.terminate = EEPROM.read((segment * FTA_CYCLE_SIZE) + 1); // Read the termination type
  current.action = EEPROM.read((segment * FTA_CYCLE_SIZE) + 2);    // Read the action type
  byte tempMSB = EEPROM.read((segment * FTA_CYCLE_SIZE) + 3);      // Read MSB of leftAmount
  byte tempLSB = EEPROM.read((segment * FTA_CYCLE_SIZE) + 4);      // Read LSB of leftAmount
  current.leftAmount = word(tempMSB,tempLSB);            // Combine MSB and LSB
  tempMSB = EEPROM.read((segment * FTA_CYCLE_SIZE) + 5); // Read MSB of righttAmount
  tempLSB = EEPROM.read((segment * FTA_CYCLE_SIZE) + 6); // Read LSB of righttAmount
  current.rightAmount = word(tempMSB,tempLSB);           // Combine MSB and LSB
  
  // Execute follow-type until termination occurs:
  
  // TODO:
  // Perform appropriate action.
  
}

// Checks to see if the robot is at a turn or a 'T', by checking the outer sensors.
// NOTE: This assumes white line on black surface.
int isTurn()
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
