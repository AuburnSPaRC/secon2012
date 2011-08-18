////code to test PING))) ultrasonic distance Sensor
//
//#include "math.h"
//
//void setup()
//{
//    Serial.begin(9600);
//}
//
//void loop()
//{
//  pinMode(A0,OUTPUT);
//  digitalWrite(A0,HIGH);
//  delayMicroseconds(12);
//  digitalWrite(A0,LOW);
//  pinMode(A0,INPUT);
//  analogRead(A0);

// this constant won't change.  It's the pin number
// of the sensor's output:
int pingPin_sen1 = 7;
//int pingPin_sen2 = 8;

void setup() {
  // initialize serial communication:
  Serial.begin(9600);
 }

void loop()
{
  // establish variables for duration of the ping, 
  // and the distance result in inches and centimeters:
  long duration_sen1, inches_sen1, cm_sen1;
  //long duration_sen2, inches_sen2, cm_sen2;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin_sen1, OUTPUT); //pinMode(pingPin_sen2, OUTPUT);
  digitalWrite(pingPin_sen1, LOW);// digitalWrite(pingPin_sen2, LOW);
  delayMicroseconds(2); 
  digitalWrite(pingPin_sen1, HIGH);// digitalWrite(pingPin_sen2, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin_sen1, LOW); //digitalWrite(pingPin_sen2, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.

  pinMode(pingPin_sen1, INPUT);
  duration_sen1 = pulseIn(pingPin_sen1, HIGH, 10000);
  inches_sen1 = microsecondsToInches(duration_sen1);
  cm_sen1 = microsecondsToCentimeters(duration_sen1);

//  pinMode(pingPin_sen2, INPUT);
//  duration_sen2 = pulseIn(pingPin_sen2, HIGH, 1000);
//  inches_sen2 = microsecondsToInches(duration_sen2);
//  cm_sen2 = microsecondsToCentimeters(duration_sen2);

  Serial.print(inches_sen1);
  Serial.print("in_sen1, ");
  Serial.print(cm_sen1);
  Serial.print("cm_sen1,   ");
//     Serial.print(inches_sen2);
//     Serial.print("in_sen2, ");
//     Serial.print(cm_sen2);
//     Serial.print("cm_sen2, ");
  Serial.println();
  delay(100);
}
   
long microsecondsToInches(long microseconds)
{
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}
