//Code by Christoper James

//For finding the shape of a voltage waveform (sawtooth or square)
//4. Interpret signal waveform; if it is a square wave, go right, if it is a saw-tooth wave, go left.
//a. For this task, the left plate will be the signal source, and the right plate will be ground
//b. Wave frequency will be on the order of 100 kHz
//c. Maximum current that can be sourced is 10mA
//d. The RMS voltage for both signals will be 5V

//This code is written assuming that the square wave goes from 0 to Vp.

#define Vmax 255    //maximum voltage level
#define num_samples 50 //number of samples
#define thresh_percent .10 //10% of reading must be NOT square to be a sawtooth wave
#define thresh_volt 0.10   //10% within the min or max = square, otherwise sawtooth

#define pin_RD 10        //pin 10 - connected to RD of the MAX153
#define pin_INT 11       //pin 11 - connected to INT of the MAX 153
#define pin_CS 12        //pin 12 - connected to CS of MAX 153

#include "math.h"

int sqcount = 0; //count for the square wave, variable for signal value
int t1,t2;

char fullRead()  //defines fullRead, the function that reads pins D0-D7 for input data
{
  char result = PIND;
  return result;
}

void setup()
{
 pinMode(pin_CS, OUTPUT); //Pin 12 will enable / disable the MAX 153
 pinMode(pin_RD, OUTPUT); //Pin 10 will be low to read in our values, and high when finished (RD)
 pinMode(pin_INT, INPUT);  //Pin 11 will read the ADC's interrupt output to see when to stop looping (INT)
 DDRD = 0;  //Sets Port D to be an input; will read D0 - D7
}

void loop()
{
  DDRD = 0;
  digitalWrite(pin_CS,LOW);
  delay(1);
  t1 = micros();
  for (int i=0; i<1000; i++)
  {
    //digitalWrite(pin_RD, LOW);  //write RD LOW to read in data
    PORTB &= B11111011;
    //while(digitalRead(pin_INT)) {} //while INT is HIGH, the code waits
    while(PORTB & B00001000) {}
    char val = fullRead();  //read D0-D7
    //digitalWrite(pin_RD, HIGH); //stop reading in data
    PORTB |= B00000100;
  }
  t2 = micros();
  
  digitalWrite(pin_CS,HIGH);
  delay(1);
  Serial.begin(9600);
  Serial.println(t2-t1);
  delay(1);
  Serial.end();
  delay(497);
}
 
  
