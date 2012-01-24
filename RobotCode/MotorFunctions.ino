// Converts the value between +/- MAX_VELOCITY to an actual PWM signal and direction
void updateMotors()
{
  double tempLeftSpeed = forwardSpeed + leftDelta;    // Left motor speed from -255 to 255
  double tempRightSpeed = forwardSpeed + rightDelta;  // Reft motor speed from -255 to 255
  
  analogWrite(LEFT_PWM_PIN, byte(abs(tempLeftSpeed)));   // Set PWM as magnitude of left speed (0 to 255)
  analogWrite(RIGHT_PWM_PIN, byte(abs(tempRightSpeed))); // Set PWM as magnitude of right speed (0 to 255)
  
  if (tempLeftSpeed < 0) {digitalWrite(LEFT_DIR_PIN, LOW);}   // If negative, direction = LOW;
  else {digitalWrite(LEFT_DIR_PIN, HIGH);}  // If positive, direction = HIGH
  
  if (tempRightSpeed < 0) {digitalWrite(RIGHT_DIR_PIN, LOW);} // If negative, direction = LOW;
  else {digitalWrite(RIGHT_DIR_PIN, HIGH);} // If positive, direction = HIGH
}
