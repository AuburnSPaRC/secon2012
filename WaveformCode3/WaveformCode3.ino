//Code by Christoper James

//For finding the shape of a voltage waveform (sawtooth or square)
//4. Interpret signal waveform; if it is a square wave, go right, if it is a saw-tooth wave, go left.
//a. For this task, the left plate will be the signal source, and the right plate will be ground
//b. Wave frequency will be on the order of 100 kHz
//c. Maximum current that can be sourced is 10mA
//d. The RMS voltage for both signals will be 5V

//This code is written assuming that the square wave goes from 0 to Vp.

#define Vsqmax 185    //maximum square wave voltage level
#define Vsqmin 47     //minimum square wave voltage level
#define num_samples 500 //number of samples
#define sq_thresh_percent .65 //75% of reading must be square to be a square wave
#define tri_thresh_percent .35
#define thresh_volt 0.15   //10% within the min or max = square, otherwise sawtooth

#define pin_RD 30        //pin 10 - connected to RD of the MAX153
#define pin_INT 32       //pin 11 - connected to INT of the MAX 153
#define pin_CS 34        //pin 12 - connected to CS of MAX 153

#define RELAY_K1_PIN   52 // (DIG) DPin 52 - Control Relay 1
#define RELAY_K2_PIN   53 // (DIG) DPin 53 - Control Relay

#include "math.h"

int sqcount = 0;
int sawcount = 0; //count for the square wave/sawtooth wave, variable for signal value
int badcount =0;
int my_status;
//byte value[num_samples];





byte fullRead()  //defines fullRead, the function that reads pins D0-D7 for input data
{
  byte result = PIND;
  return result;
}

void setup()
{
    pinMode(RELAY_K1_PIN, OUTPUT);
  pinMode(RELAY_K2_PIN, OUTPUT);
    digitalWrite(RELAY_K1_PIN, 1);
  digitalWrite(RELAY_K2_PIN, 1);
  delay(1000);
    digitalWrite(RELAY_K1_PIN, 0);
  digitalWrite(RELAY_K2_PIN, 0);
 pinMode(pin_CS, OUTPUT); //Pin 12 will enable / disable the MAX 153
 pinMode(pin_RD, OUTPUT); //Pin 10 will be low to read in our values, and high when finished (RD)
 pinMode(pin_INT, INPUT);  //Pin 11 will read the ADC's interrupt output to see when to stop looping (INT)
 DDRD = 0;  //Sets Port D to be an input; will read D0 - D7
   Serial.begin(9600);
}

void loop()
{
  byte val = 0;
  sqcount = 0;
  sawcount = 0;
  badcount=0;
  DDRD = 0;  //PORT D as input
//  int t1,t2;  //used to test timing
  digitalWrite(pin_CS,LOW);
  delay(1);
//  t1 = micros();
  for (int i=0; i<num_samples; i++)
  {
    unsigned int blah = 0;
    val = 0;
    //digitalWrite(pin_RD, LOW);  //write RD LOW to read in data
    PORTB &= B11111011;
    //while(digitalRead(pin_INT)) {} //while INT is HIGH, the code waits
    while(PORTB & B00001000) {}
    delayMicroseconds(4);
    val = fullRead();  //read D0-D7
//  value[i] = val;
    blah = (unsigned int)val;
    //digitalWrite(pin_RD, HIGH); //stop reading in data
    PORTB |= B00000100;
    if ((blah*1.0 >= (Vsqmax - Vsqmax*thresh_volt)) && 
        (blah*1.0 <= (Vsqmax + Vsqmax*thresh_volt)))  //if voltage is within 10% of square wave maximum
    {
      sqcount++;
    }
    else if ((blah*1.0 >= (Vsqmin - Vsqmax*thresh_volt)) && 
             (blah*1.0 <= (Vsqmin + Vsqmax*thresh_volt)))  //if voltage is within 10% of square wave minimum
    {
      sqcount++;
    }
    else  //if voltage is neither, implying that it must be inbetween 
    {
      sawcount++;
    } 
//    else
//    {
//      badcount++;
//    }
      
  }
//  t2 = micros();
  if (sqcount > num_samples*(sq_thresh_percent))   //if x% of samples are within range, then it must be a square wave
  {
    my_status = 1;//Turn to the right
  }
  else if (sawcount > num_samples*(tri_thresh_percent))   //if x% of samples are NOT within range, then it must be a sawtooth wave
  {
    my_status = 2;//Turn to the left
  }
  else  //if for some reason neither fit
  {
    my_status = 0;//ERROR!
  }
  
  digitalWrite(pin_CS,HIGH);
  delay(1);
  delay(1);
//  Serial.println(Vsqmax - Vsqmax*thresh_volt);
//  Serial.println(Vsqmax + Vsqmax*thresh_volt);
//  Serial.println(Vsqmin - Vsqmax*thresh_volt);
//  Serial.println(Vsqmin + Vsqmax*thresh_volt);
  Serial.print("Square: ");
  Serial.println(sqcount);
  Serial.print("Saw: ");
  Serial.println(sawcount);
  Serial.print("Bad: ");
  Serial.println(badcount);
  Serial.print("status");
  Serial.println(my_status);
//  Serial.print("time: ");
//  Serial.println(t2-t1);
  Serial.println("-------------");
  
//  for (int i = 0; i < num_samples; i++)
//  {
//    Serial.print(i);
//    Serial.print(",");
//    Serial.print((unsigned int)value[i]);
//    Serial.println(",");
//  }
//  for (int i = 0; i < badcount; i++)
//  {
//    Serial.print(i);
//    Serial.print(",");
//    Serial.print((unsigned int)bad[i]);
//    Serial.println(",");
//  }
  delay(1);
  delay(20000);
}
 
  
