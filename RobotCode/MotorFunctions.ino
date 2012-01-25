/**  Code for the controlling the two motors of the robot.
* 
* Allows for control of two motors independently using a PWM
* signal and a direction bit for each.
*
* Credits: Tyler Crumpton
*
*/

// --- Motor Pin Definitions ---
#ifndef OVERWRITE_SENSOR_PINS
  #define LEFT_PWM_PIN   10  // (PWM) DPin 10  - Left motor PWM speed control
  #define LEFT_DIR_PIN   11  // (DIG) DPin 11  - Left motor direction control
  #define RIGHT_PWM_PIN  12  // (PWM) DPin 12  - Right motor PWM speed control 
  #define RIGHT_DIR_PIN  13  // (DIG) DPin 13  - Right motor direction control
#endif

// Converts the value between +/- MAX_VELOCITY to an actual PWM signal and direction
void updateMotors()
{
  double tempLeftSpeed  = forwardSpeed + leftDelta;   // Left motor speed from -255 to 255
  double tempRightSpeed = forwardSpeed + rightDelta;  // Reft motor speed from -255 to 255
  
  analogWrite(LEFT_PWM_PIN, byte(abs(tempLeftSpeed)));   // Set PWM magnitude of left speed (0 to 255)
  analogWrite(RIGHT_PWM_PIN, byte(abs(tempRightSpeed))); // Set PWM magnitude of right speed (0 to 255)
  
  if (tempLeftSpeed < 0)
    digitalWrite(LEFT_DIR_PIN, LOW);   // If negative, direction = LOW
  else 
    digitalWrite(LEFT_DIR_PIN, HIGH);  // If positive, direction = HIGH
  
  if (tempRightSpeed < 0)
    digitalWrite(RIGHT_DIR_PIN, LOW);  // If negative, direction = LOW
  else 
    digitalWrite(RIGHT_DIR_PIN, HIGH); // If positive, direction = HIGH
}
