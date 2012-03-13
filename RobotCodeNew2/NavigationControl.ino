// Termination types:
#define ON_LINE       0 // Bot is on the line
#define OFF_LINE      1 // Bot is off the line
#define HIT_SWITCH    2 // Physical switch is triggered
#define AT_T          3 // Bot is at a T-intersection
#define AT_LEFT       4 // Bot is at left turn
#define AT_RIGHT      5 // Bot is at right turn
#define AT_CENTER     6 // Bot is at the center
#define NOWHERE       -1// Bot is nowhere


// Action types:
#define TURN_IN_PLACE   0 // Rotate left or right in place
#define LEFT_THEN_RIGHT 1 // Turn left wheel then right wheel
#define RIGHT_THEN_LEFT 2 // Turn right wheel then left wheel
#define DO_NOTHING      3 // Do nothing

//A global variable used to store the current termination
int atTermination;
int delayer;        //Used to delay the checking of the termination types after the start of each section to give the robot time to get off the line

// Executes the three-stage cycle (follow, terminate, action) for a given segment of course.
void executeSegment(int segment)
{
  int nOccur=0;
  
  delayer=0;                //reset the delayer
  atTermination=NOWHERE;    //reset our termination
  /*
    Serial.print("\n");
    Serial.print("Follow:");
    Serial.print((courseConfig[segment].follow));
    Serial.print("\n");
    Serial.print("Terminate:");
    Serial.print((courseConfig[segment].termination));
    Serial.print("\n");       
    Serial.print("Action:");  
    Serial.print((courseConfig[segment].action));
    Serial.print("\n");
    Serial.print("Left Amount:");  
    Serial.print((courseConfig[segment].leftAmount));
    Serial.print("\n");   
    Serial.print("Right Amount:");  
    Serial.print((courseConfig[segment].rightAmount));
    Serial.print("\n");   //Debugging print out
    Serial.print("Speed:");  
    Serial.print((courseConfig[segment].bot_speed));
    Serial.print("\n");   //Debugging print out
    Serial.print("Turn Speed:");  
    Serial.print((courseConfig[segment].turn_speed));
    Serial.print("\n");   //Debugging print out  
    Serial.print("Center:");  
    Serial.print((courseConfig[segment].center));
    Serial.print("\n");   //Debugging print out  
    Serial.print("KP: ");  
    Serial.println(courseConfig[segment].KP,5);
    Serial.print("KI: ");  
    Serial.println(courseConfig[segment].KI,5);
    Serial.print("KD: ");  
    Serial.println(courseConfig[segment].KD,5);
    Serial.print("Ignore: ");  
    Serial.println(courseConfig[segment].skip_section);
    Serial.print("Clicks: ");  
    Serial.println(courseConfig[segment].clicks);
    Serial.print("Occurances: ");  
    Serial.println(courseConfig[segment].occurance);  
   Serial.print("\n"); //Debugging print out*/
 
  //First do the actual part
  if(courseConfig[segment].follow==0)    //Line following
  {
    //Set up our PID for this section
    setpointPID=courseConfig[segment].center*100;
    FULL_SPEED=float(int(courseConfig[segment].bot_speed)/255.0);
    TURN_SPEED=float(int(courseConfig[segment].turn_speed)/255.0);
    MAX_DELTA = FULL_SPEED*MAX_VELOCITY;

    lfPID.SetOutputLimits(-MAX_DELTA, MAX_DELTA); // force PID to the range of motor speeds.   
    lfPID.SetTunings(courseConfig[segment].KP, courseConfig[segment].KI, courseConfig[segment].KD); // Set the PID loops tuning values to the new ones from EEPROM
    lfPID.SetMode(AUTOMATIC); // Turn on PID    

    while(true)
    {
      
      followLine();
      if(delayer>100)checkTermination();
      else delayer++;
      if(atTermination==courseConfig[segment].termination){nOccur++;delayer=0;if(nOccur>=courseConfig[segment].occurance){break;}}
    }
    lfPID.SetMode(MANUAL);
  }
  else      //Encoders
  {
    while(true)
    {
      encoderMove(5);
      if(delayer>100)checkTermination();
      else delayer++;
      if(atTermination==courseConfig[segment].termination){nOccur++;delayer=0;if(nOccur>=courseConfig[segment].occurance){break;}}
    }
  }
  
  
  //Turn left then right
  if(courseConfig[segment].action==LEFT_THEN_RIGHT)
  {
    if(!goLeft)turnLeftThenRight(courseConfig[segment].rightAmount,courseConfig[segment].leftAmount,true);
    else turnLeftThenRight(courseConfig[segment].leftAmount,courseConfig[segment].rightAmount,false);
  }
  else if(courseConfig[segment].action==RIGHT_THEN_LEFT)    //Turn right then left
  {
    if(!goLeft)turnLeftThenRight(courseConfig[segment].rightAmount,courseConfig[segment].leftAmount,false);
    else turnLeftThenRight(courseConfig[segment].leftAmount,courseConfig[segment].rightAmount,true);
  }  
  else if(courseConfig[segment].action==TURN_IN_PLACE)
  {
    if(!goLeft)turnInPlace(courseConfig[segment].leftAmount);
    else turnInPlace(-courseConfig[segment].leftAmount);
  }
}
  

// Checks to see if the robot is at a turn or a 'T', by checking the outer sensors.
// NOTE: This assumes white line on black surface.
int returnPlaceOnLine(void)
{
  int minSensor=-1,numOnLine=0;
  int minValue=1000;
  fSensors.readCalibrated(fSensorValues, QTR_EMITTERS_ON);
  
 for(int i=0;i<NUM_SENSORS;i++)
  {
 /*    Serial.print("Sensor[");
     Serial.print(i);
     Serial.print("]: ");
     Serial.print(fSensorValues[i]);
     Serial.print("\n\n");*/
    if(fSensorValues[i]<=minValue){minSensor=i;minValue=fSensorValues[i];}
    if(fSensorValues[i]<=750){numOnLine++;}
  }
  
  if(numOnLine<=3){return minSensor;}
  else return -1;
}
     
  

// Checks to see if the robot is at a turn or a 'T', by checking the outer sensors.
// NOTE: This assumes white line on black surface.
int checkTermination(void)
{
  
  fSensors.readCalibrated(fSensorValues, QTR_EMITTERS_ON);


  boolean isRight = (fSensorValues[NUM_SENSORS-1] < REFLECT_THRESHOLD);    //&&(fSensorValues[(NUM_SENSORS)-2] < REFLECT_THRESHOLD));
  boolean isLeft  = (fSensorValues[0]  < REFLECT_THRESHOLD);
  
  if (isRight){atTermination=AT_RIGHT;return 1;}
  if (isLeft){atTermination=AT_LEFT;return 1;}
  
  atTermination=NOWHERE;
  return 0;
}
