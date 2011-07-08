//Code by Christoper James

//For finding the shape of a voltage waveform (sawtooth or square)
//4. Interpret signal waveform; if it is a square wave, go right, if it is a saw-tooth wave, go left.
//a. For this task, the left plate will be the signal source, and the right plate will be ground
//b. Wave frequency will be on the order of 100 kHz
//c. Maximum current that can be sourced is 10mA
//d. The RMS voltage for both signals will be 5V

//This code is written assuming that the square wave goes from 0 to Vp.

#define Vsqmax 201    //maximum square wave voltage level
#define Vsqmin 54     //minimum square wave voltage level
#define num_samples 1000 //number of samples
#define thresh_percent .10 //10% of reading must be NOT square to be a sawtooth wave
#define thresh_volt 0.20   //10% within the min or max = square, otherwise sawtooth

#define pin_RD 10        //pin 10 - connected to RD of the MAX153
#define pin_INT 11       //pin 11 - connected to INT of the MAX 153
#define pin_CS 12        //pin 12 - connected to CS of MAX 153

#include "math.h"

int sqcount = 0;
int sawcount = 0; //count for the square wave/sawtooth wave, variable for signal value
int my_status;

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
  sqcount = 0;
  sawcount = 0;
  DDRD = 0;
  digitalWrite(pin_CS,LOW);
  delay(1);
  for (int i=0; i<num_samples; i++)
  {
    //digitalWrite(pin_RD, LOW);  //write RD LOW to read in data
    PORTB &= B11111011;
    //while(digitalRead(pin_INT)) {} //while INT is HIGH, the code waits
    while(PORTB & B00001000) {}
    char val = fullRead();  //read D0-D7
    //digitalWrite(pin_RD, HIGH); //stop reading in data
    PORTB |= B00000100;
    if (val >= (Vsqmax - Vsqmax*thresh_volt) && val <= (Vsqmax + Vsqmax*thresh_volt))  //if voltage is within 10% of square wave maximum
    {
      sqcount++;
    }
    else if (val >= (Vsqmin - Vsqmax*thresh_volt) && val <= (Vsqmin + Vsqmax*thresh_volt))  //if voltage is within 10% of square wave minimum
    {
      sqcount++;
    }
    else   //if voltage is neither, implying that it must be inbetween 
    {
      sawcount++;
    } 
  }
  if (sqcount >= num_samples*(1 - thresh_percent))   //if 90% of samples are within range, then it must be a square wave
  {
    my_status = 1;//Turn to the right
  }
  else if (sawcount >= num_samples*(1 - thresh_percent))   //if 90% of samples are NOT within range, then it must be a sawtooth wave
  {
    my_status = 2;//Turn to the left
  }
  else  //if for some reason neither fit
  {
    my_status = 0;//ERROR!
  }
  
  digitalWrite(pin_CS,HIGH);
  delay(1);
  Serial.begin(9600);
  delay(1);
  Serial.print("Square: ");
  Serial.println(sqcount);
  Serial.print("Saw: ");
  Serial.println(sawcount);
  Serial.print("status");
  Serial.println(my_status);
  Serial.println("-------------");
  delay(1);
  Serial.end();
  delay(2000);
}
 
  
