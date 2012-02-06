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
//      leftDelta=0;
//      rightDelta=0;
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
  //TODO: Implement better turning  
  setMove(TURN_LEFT);
  delay(TURN_TIME);
}

// Turn bot 90 degrees to the right
void turnRight()
{
  //TODO: Implement better turning
  setMove(TURN_RIGHT);
  delay(TURN_TIME);
}

//Moves the bot up to the task sensors for a reading
void moveToSensor()
{
  //TODO: Add actual implementation
  setMove(STOP);
  delay(3000);
}

//Moves the bot back from the task to the 'T'
void moveFromSensor()
{
  //TODO
}

// Spins each wheel 10 times for speed measurement.
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
}
