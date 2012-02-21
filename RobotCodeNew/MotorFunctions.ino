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
  #define LEFT_PWM_PIN   5  // Green(PWM) DPin 10  - Left motor PWM speed control
  #define LEFT_DIR_PIN   38  // (DIG) DPin 11  - Left motor direction control
  #define LEFT_EN_PIN    36  // (DIG) DPin 12  - Left motor enable control
  #define RIGHT_PWM_PIN  2  // (PWM) DPin 13  - Right motor PWM speed control 
  #define RIGHT_DIR_PIN  3  // (DIG) DPin 14  - Right motor direction control
  #define RIGHT_EN_PIN   4  // Blue(DIG) DPin 15  - Right motor enable control
#endif

// Converts the value between +/- MAX_VELOCITY to an actual PWM signal and direction
void updateMotors()
{
  double tempLeftSpeed  = forwardSpeed + leftDelta;   // Left motor speed from -255 to 255
  double tempRightSpeed = forwardSpeed + rightDelta;  // Reft motor speed from -255 to 255
  
  #ifdef DEBUG_MOTOR_SPEED
    Serial.print("LeftD: ");
    Serial.print(leftDelta);
    Serial.print("\tRightD: ");
    Serial.print(rightDelta);
    
    Serial.print("\t\tLeft: ");
    Serial.print(tempLeftSpeed);
    Serial.print("\tRight: ");
    Serial.println(tempRightSpeed);
  #endif
  
  digitalWrite(LEFT_EN_PIN, HIGH);   // Left motor enable
  digitalWrite(RIGHT_EN_PIN, HIGH);  // Right motor enable
  
  if (tempLeftSpeed < 0)
  {
    digitalWrite(LEFT_DIR_PIN, LOW);       // If negative, direction = LOW
    tempLeftSpeed = byte(abs(tempLeftSpeed)); // PWM=255 means full speed when DIR=LOW
  }
  else 
  {
    digitalWrite(LEFT_DIR_PIN, HIGH);      // If positive, direction = HIGH
    tempLeftSpeed = byte(abs(tempLeftSpeed)); // Convert to unsigned byte
    tempLeftSpeed = 255 - tempLeftSpeed;   // PWM=0 means full speed when DIR=HIGH
  }
  
  if (tempRightSpeed < 0)
  {
    digitalWrite(RIGHT_DIR_PIN, LOW);      // If negative, direction = LOW
    tempRightSpeed = byte(abs(tempRightSpeed)); // PWM=255 means full speed when DIR=LOW
    tempRightSpeed += MOTOR_OFFSET;
  }
  else 
  {
    digitalWrite(RIGHT_DIR_PIN, HIGH);     // If positive, direction = HIGH
    tempLeftSpeed = byte(abs(tempLeftSpeed)); // Convert to unsigned byte
    tempRightSpeed = 255 - tempRightSpeed; // PWM=0 means full speed when DIR=HIGH
    tempRightSpeed -= MOTOR_OFFSET;
  }
  
  analogWrite(LEFT_PWM_PIN, tempLeftSpeed);   // Set PWM magnitude of left speed (0 to 255)
  analogWrite(RIGHT_PWM_PIN, tempRightSpeed); // Set PWM magnitude of right speed (0 to 255)
}
