// made by Ben Straub
// made to find the value of a capacitor
/*
  III.
  3. Measure the capacitance between two plates; if the capacitance is greater than 550 nF, then go 
  right, if it is less than 450 nF, go left.
    a. Capacitors will be bidirectional
    b. Maximum capacitance is 10µF
    c. Minimum capacitance is 10nF
    d. Maximum voltage tolerance is 12V
*/  

// use this diagram:
//
//        ,----------o A0 / pinCR1
//        |
//        |   10 Ohm
//        |---^^^----o D2 / pinCR2
//        |
//        |   500 Ohm
//    ,---'---^^^----o D3 / pinCR3
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

#define pinCR1 A0 //samples the cap voltage (analog)
#define pinCR2 2  //discharges the capacitor (digital)
#define pinCR3 3  //charges the capacitor (digital)

#define MidCap 0.5 //uF
#define R 500.0

#include "math.h"
#include "status.h"

Status CapRead();  //function prototype

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  
  CapRead();
  delay(1000);
}

Status CapRead()
{
  //declare variables
  int V1, V2;  //holds voltage samples
  float C;     //holds the calculated capacitance (in uF)
  
  //setup
  pinMode(pinCR2,OUTPUT);
  pinMode(pinCR3,OUTPUT);
  digitalWrite(pinCR1,LOW);
  pinMode(pinCR1,INPUT);
  digitalWrite(pinCR2,LOW);
  
  digitalWrite(pinCR3,LOW); //discharge cap
  delay(1);  //make sure it's fully discharged
  pinMode(pinCR3,INPUT); //stop discharging by setting this pin to a high impedance state
  delayMicroseconds(4);
  digitalWrite(pinCR2,HIGH); //start charging
  V1 = analogRead(pinCR1); // get first reading
  V2 = analogRead(pinCR1); // occurs 112us after first reading
  
  C = 112.0 / (R * log(float(1023-V1)/float(1023-V2)));
  // ^ fancy mathematics
  
  if (V1 >= 900)
  { return ERROR; }
  else if (V2 == 1023) //cap is too small
  { return LEFT; }
  else if (V1 == V2)  //cap is too big
  { return RIGHT; }
  else if (C > MidCap)
  { return RIGHT; }
  else
  { return LEFT; }
} 
