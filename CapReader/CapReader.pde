// made by Ben Straub
// made to find the value of a capacitor
// use this diagram:
//
//        ,----------o A0
//        |
//        |   10 Ohm
//        |---^^^----o D0
//        |
//        |   500 Ohm
//    ,---'---^^^----o D1
//    |
//   === unknkown capacitor
//    |
//    '--------------o GND
//
// This is specifically designed for an unknown cap around
// 0.5 uF in value.
// This was designed for my RBBB arduino clone and uses the
// fact that an analogRead() takes about 112us to exectue
// to calculate timing.  Using a board with a different
// sample speed will mess up this calculation.

#define R 500.0

#include "math.h"

int V1, V2;
float C;
//unsigned long t1,t2;

void setup()
{
  Serial.begin(9600);
  pinMode(2,OUTPUT);
}

void loop()
{
  pinMode(3,OUTPUT);
  digitalWrite(2,LOW);
  digitalWrite(3,LOW); //discharge cap
  delay(1);  //make sure it's fully discharged
  pinMode(3,INPUT); //stop discharging
  delayMicroseconds(4);
  digitalWrite(2,HIGH); //start charging
  V1 = analogRead(A0); // get first reading
  //t1 = micros();
  V2 = analogRead(A0); // occurs 112us after first reading
  //t2=micros();
  C = 112.0 / (R * log(float(1023-V1)/float(1023-V2)));
  // ^ fancy mathematics
  Serial.print("V1 was ");
  Serial.println(V1);
  Serial.print("V2 was ");
  Serial.println(V2);
  Serial.print("C was ");
  Serial.println(C,4); //show C (in uF) to 4 decimal places.
  //Serial.print("time: ");
  //Serial.println(t2-t1);
  
  delay(1000);
}
