/**  Code for the controlling the two motors of the robot.
* 
* Allows for control of two motors independently using a PWM
* signal and a direction bit for each.
*
* Credits: Tyler Crumpton
*
*/


void updateMotors()
{
  double tempLeftSpeed  = forwardSpeed + leftDelta;   // Left motor speed from -255 to 255
  double tempRightSpeed = forwardSpeed + rightDelta;  // Reft motor speed from -255 to 255
  if(tempLeftSpeed>255)tempLeftSpeed=FULL_SPEED*MAX_VELOCITY;
  else if(tempLeftSpeed<-FULL_SPEED*MAX_VELOCITY)tempLeftSpeed=-FULL_SPEED*MAX_VELOCITY;
  
  if(tempRightSpeed>FULL_SPEED*MAX_VELOCITY)tempRightSpeed=FULL_SPEED*MAX_VELOCITY;
  else if(tempRightSpeed<-FULL_SPEED*MAX_VELOCITY)tempRightSpeed=-FULL_SPEED*MAX_VELOCITY;  
  
  
  /*  Serial.print("LeftD: ");
    Serial.print(leftDelta);
    Serial.print("\tRightD: ");
    Serial.print(rightDelta);
    
    Serial.print("\t\tLeft: ");
    Serial.print(tempLeftSpeed);
    Serial.print("\tRight: ");
    Serial.println(tempRightSpeed);*/

  
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



/*
// Does PID for line folliwing and sets the motor delta speeds.
void followLineHung()
{
  // Read calibrated front sensor values and obtain a measure of the line position from 0 to NUM_SENSORS-1
  unsigned int pOsitIon = fSensors.readLine(fSensorValues,QTR_EMITTERS_ON,1);
//  Serial.println(courseConfig[location].center);

  inputPID = pOsitIon;            // set PID input to position of line
  
  lfPID.Compute();                // compute correction, store in outputPID
    Serial.println(outputPID);
  if (outputPID > 30)
  {
    setMove(STOP_LEFT);
  }
  else if(outputPID<-30)
  {
    setMove(STOP_RIGHT);
  }
}
*/
