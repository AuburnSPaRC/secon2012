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
  //TODO
}

// Turn bot 90 degrees to the right
void turnRight()
{
  //TODO
}

//Moves the bot up to the task sensors for a reading
void moveToSensor()
{
  //TODO
}

//Moves the bot back from the task to the 'T'
void moveFromSensor()
{
  //TODO
}
