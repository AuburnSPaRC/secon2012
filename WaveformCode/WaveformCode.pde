//Code by Christoper James

//For finding the shape of a voltage waveform (sawtooth or square)
//4. Interpret signal waveform; if it is a square wave, go right, if it is a saw-tooth wave, go left.
//a. For this task, the left plate will be the signal source, and the right plate will be ground
//b. Wave frequency will be on the order of 100 kHz
//c. Maximum current that can be sourced is 10mA
//d. The RMS voltage for both signals will be 5V

//This code is written assuming that the square wave goes from 0 to Vp.

#define F 2000000.00 //frequency
#define T 0.0000005  //period
#define Vmax 1023    //maximum voltage level

#include "math.h"

int sqcount, sawcount, signal, i; //counts for square wave and sawtooth wave checks, signal is the voltage value

void setup()
{
 Serial.begin(9600); 
 pinMode(4, INPUT);  //Pin 4 will read in voltage levels
 pinMode(5, OUTPUT); //Pin 5 will send a HIGH if the waveform is a square wave
 pinMode(6, OUTPUT); //Pin 6 will send a HIGH if the waveform is a sawtooth wave
}

void loop()
{
  for(i = 0; i < 50; i++) //for loop with sample the signal 50 times during a 0.5 us period
  {  
    signal = analogRead(A1);
    if (signal >= Vmax*.=0.9 && signal <= Vmax*1.1)  //if voltage is within 10% of square wave maximum
    {
      sqcount++, i++;
    }
    else if (signal >= 0 && signal <= Vmax*0.1)  //if voltage is within 10% of square wave minimum
    {
      sqcount++, i++;
    }
    else  //if voltage is neither, implying that it must be inbetween
    {
      sawcount++, i++;
    }
    delay(0.01); //delay 1/50 of the period to get 50 readings
  }
  if (sqcount > sawcount) //if it is a square wave, an LED at 5 would turn on, for example
  {
    digitalWrite(5, HIGH);
  }
  else if (sawcount > sqcount)  //if it is a sawtooth wave, 6 would turn on instead
  {
    digitalWrite(6, HIGH);
  }
  else  //if there is some sort of error or issue, both will turn on
  {
    digitalWrite(5, HIGH);
    digitalWrite(6, HIGH);
  }
}
  
  
