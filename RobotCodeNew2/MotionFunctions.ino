/**  Code for the defining the motion of the robot.
* 
* Defines the motions and trajectories that the robot 
* may make on the course. Included motions are turning,
* forward/backward, approach task, leave task, etc.
*
* Depends: MotorFunctions.ino
*
* Credits: Tyler Crumpton, Levi Smolin
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
#define AT_CENTER     7 // Bot is at the center



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
    case MOVE_FAST:
      forwardSpeed = MAX_VELOCITY;
      leftDelta    = 0;
      rightDelta   = 0;
      break;
    case STOP_LEFT:
      forwardSpeed=0;
      leftDelta=0;
      rightDelta=FULL_SPEED*MAX_VELOCITY;
      break;
    case STOP_RIGHT:
      forwardSpeed=0;
      rightDelta=0;
      leftDelta=FULL_SPEED*MAX_VELOCITY;
      break;
    default:
      forwardSpeed = FULL_SPEED*MAX_VELOCITY;
      leftDelta    = 0;
      rightDelta   = 0;
  }
  updateMotors();
}





