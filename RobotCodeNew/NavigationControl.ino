#include <EEPROM.h>

#define FTA_CYCLE_SIZE  10 // Defines the size (in bytes) of the ftaCycle struct.

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

// Action types:
#define TURN_IN_PLACE   0 // Rotate left or right in place
#define LEFT_THEN_RIGHT 1 // Turn left wheel then right wheel
#define RIGHT_THEN_LEFT 2 // Turn right wheel then left wheel


// Executes the three-stage cycle (follow, terminate, action) for a given segment of course.
void executeSegment(int segment)
{
  u_double tempVals[3];    //Temporary structs to read in the floats
  
  // --- Retrieve FTA information from EEPROM: ---
  courseConfig[location].follow = EEPROM.read(segment * FTA_CYCLE_SIZE);          // Read the follow type
  courseConfig[location].terminate = EEPROM.read((segment * FTA_CYCLE_SIZE) + 1); // Read the termination type
  courseConfig[location].action = EEPROM.read((segment * FTA_CYCLE_SIZE) + 2);    // Read the action type
  courseConfig[location].leftAmount = EEPROM.read((segment * FTA_CYCLE_SIZE) + 3); // Read leftAmount
  courseConfig[location].rightAmount = EEPROM.read((segment * FTA_CYCLE_SIZE) + 4); // Read leftAmount
  courseConfig[location].bot_speed = EEPROM.read((segment * FTA_CYCLE_SIZE) + 5); // Read LSB of rightAmount
  courseConfig[location].turn_speed = EEPROM.read((segment * FTA_CYCLE_SIZE) + 6); // Read LSB of rightAmount  
  courseConfig[location].center = EEPROM.read((segment * FTA_CYCLE_SIZE) + 7); // Read LSB of rightAmount  
  
  tempVals[0].b[0]=EEPROM.read((segment * FTA_CYCLE_SIZE) + 8);
  tempVals[0].b[1]=EEPROM.read((segment * FTA_CYCLE_SIZE) + 9);  
  tempVals[0].b[2]=EEPROM.read((segment * FTA_CYCLE_SIZE) + 10);
  tempVals[0].b[3]=EEPROM.read((segment * FTA_CYCLE_SIZE) + 11);  
  
  tempVals[1].b[0]=EEPROM.read((segment * FTA_CYCLE_SIZE) + 12);
  tempVals[1].b[1]=EEPROM.read((segment * FTA_CYCLE_SIZE) + 13);  
  tempVals[1].b[2]=EEPROM.read((segment * FTA_CYCLE_SIZE) + 14);
  tempVals[1].b[3]=EEPROM.read((segment * FTA_CYCLE_SIZE) + 15);  

  tempVals[2].b[0]=EEPROM.read((segment * FTA_CYCLE_SIZE) + 16);
  tempVals[2].b[1]=EEPROM.read((segment * FTA_CYCLE_SIZE) + 17);  
  tempVals[2].b[2]=EEPROM.read((segment * FTA_CYCLE_SIZE) + 18);
  tempVals[2].b[3]=EEPROM.read((segment * FTA_CYCLE_SIZE) + 19);    
  
  courseConfig[location].KP=tempVals[0].dval;
  courseConfig[location].KI=tempVals[1].dval;
  courseConfig[location].KD=tempVals[2].dval;  


  setpointPID=courseConfig[location].center*100;
  FULL_SPEED=float(int(courseConfig[location].bot_speed)/255.0);
  TURN_SPEED=float(int(courseConfig[location].turn_speed)/255.0);
    
  
  
  #ifdef DEBUG_ROBOT
    Serial.print("\n");
    Serial.print("Follow:");
    Serial.print((courseConfigs[location].follow));
    Serial.print("\n");
    Serial.print("Terminate:");
    Serial.print((courseConfigs[location].terminate));
    Serial.print("\n");       
    Serial.print("Action:");  
    Serial.print((courseConfigs[location].action));
    Serial.print("\n");
    Serial.print("Left Amount:");  
    Serial.print((courseConfigs[location].leftAmount));
    Serial.print("\n");   
    Serial.print("Right Amount:");  
    Serial.print((courseConfigs[location].rightAmount));
    Serial.print("\n");   //Debugging print out
    Serial.print("Speed:");  
    Serial.print((courseConfigs[location].bot_speed));
    Serial.print("\n");   //Debugging print out
    Serial.print("Turn Speed:");  
    Serial.print((courseConfigs[location].turn_speed));
    Serial.print("\n");   //Debugging print out  
    Serial.print("Center:");  
    Serial.print((courseConfigs[location].center));
    Serial.print("\n");   //Debugging print out  
    Serial.print("KP");  
    Serial.println((courseConfigs[location].KP,5));
    Serial.print("\n");   //Debugging print out  
    Serial.print("KI");  
    Serial.println((courseConfigs[location].KI,5));
    Serial.print("\n");   //Debugging print out  
    Serial.print("KD");  
    Serial.println((courseConfigs[location].KD,5));
    Serial.print("\n");   //Debugging print out
  #endif    
  
  
  // --- Execute line- or encoder-following: ---
  if (courseConfig[location].follow == LINE_FOLLOW) // Line-following type movement
  {
    // Execute linefollowing until termination occurs:
    lfPID.SetTunings(courseConfig[location].KP, courseConfig[location].KI, courseConfig[location].KD); // Set the PID loops tuning values to the new ones from EEPROM
    lfPID.SetMode(AUTOMATIC); // Turn on PID
    
    int terminationType = checkTermination();  //Check termination type
    
    if(courseConfig[location].terminate == AT_ANY) // Special case for AT_ANY
    {
      while (terminationType < AT_ANY)
      {

        followLine();
        terminationType = checkTermination();
      }
    }
    else // All other termination types
    {
      while (terminationType != courseConfig[location].terminate)
      {
        setMove(MOVE_FORWARD); // Begin moving forward
        if(courseConfig[location].terminate!=HIT_SWITCH){followLine();}
        terminationType = checkTermination();
      }
    }
    setMove(STOP);
    lfPID.SetMode(MANUAL); // Turn off PID
  }
  else  // Encoder-travel type movement
  {
    // Execute encoder-following until termination occurs:
    encoderMoveToTerminate(courseConfig[location].terminate);
  }

  // --- Perform appropriate action: ---
  if (goLeft) // Swap directions if going left from task  //CHANGE TO goLeft
  { 
    // Swap left and right amounts:
    int temp = courseConfig[location].leftAmount;
    courseConfig[location].leftAmount = courseConfig[location].rightAmount;
    courseConfig[location].rightAmount = temp;
    
    // Swap LEFT_THEN_RIGHT for RIGHT_THEN_LEFT and vice versa:
    if (courseConfig[location].action == LEFT_THEN_RIGHT)
    {
      courseConfig[location].action = RIGHT_THEN_LEFT;
    }
    else if (courseConfig[location].action == RIGHT_THEN_LEFT)
    {
      courseConfig[location].action = LEFT_THEN_RIGHT;
    }
  }
  
  if (courseConfig[location].action == TURN_IN_PLACE)
  {
    if (goLeft) // Swap direction if turning left from box
      turnInPlace(-(courseConfig[location].leftAmount));
    else // No swap needed for going right from box
      turnInPlace(courseConfig[location].leftAmount);
  }
  else if (courseConfig[location].action == LEFT_THEN_RIGHT)
  {
    turnLeftAndRight(courseConfig[location].leftAmount, courseConfig[location].rightAmount, false); // False = turn left wheel first
  }
  else if (courseConfig[location].action == RIGHT_THEN_LEFT)
  {
    turnLeftAndRight(courseConfig[location].leftAmount, courseConfig[location].rightAmount, true); // True = turn right wheel first
  }
}

// Checks to see if the robot is at a turn or a 'T', by checking the outer sensors.
// NOTE: This assumes white line on black surface.
int checkTermination()
{
  
  fSensorRight.readCalibrated(fSensorValuesRight, QTR_EMITTERS_ON);
  fSensorLeft.readCalibrated(fSensorValuesLeft, QTR_EMITTERS_ON); 
    for(int i=0;i<NUM_SENSORS;i++)
  {
    fSensorValuesBoth[i]=fSensorValuesLeft[i];
  }
  for(int i=NUM_SENSORS;i<NUM_SENSORS*2;i++)
  {
    fSensorValuesBoth[i]=fSensorValuesRight[i-NUM_SENSORS];
    Serial.print("Sensor[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.print(fSensorValuesBoth[i]);
    if(i==NUM_SENSORS-1)Serial.print("\n");
  }
  
  
  boolean isLeft  = ((fSensorValuesBoth[0] < REFLECT_THRESHOLD)&&(fSensorValuesBoth[1] < REFLECT_THRESHOLD));
  boolean isRight = ((fSensorValuesBoth[(NUM_SENSORS*2)-1] < REFLECT_THRESHOLD)&&(fSensorValuesBoth[(NUM_SENSORS*2)-2] < REFLECT_THRESHOLD));
  boolean isOff = true;
  boolean hitSwitch = (digitalRead(HIT_SWITCH_PIN)==LOW);


  
  // Checks to see if every sensor is above threshold:
  for (int i = 0; i < NUM_SENSORS*2; i++)
  {
    isOff &= (fSensorValuesBoth[i] >= REFLECT_THRESHOLD);
  }
  
  if (hitSwitch)
    return (HIT_SWITCH);
  else if (isLeft && isRight) 
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
