/**********************************************************************
*    Code is a collaboration of work by Ben Straub, Christopher James, 
*    Tyler Crumpton, Kayla Frost, and Ray Preston.
*
*    This code is for the controlling the sensors that read the four
*    task boxes.
*    The device responds to character messages sent via serial, sending
*    a response back over the serial.  For documentation on the list of
*    transmitted character, check the wiki on redmine.
*    For a schematic of the circuit this code is intended for, check
*    redmine.
**********************************************************************/

////////////////////////
//  PIN BUDGET        //
////////////////////////
// D0-D7     Waveform       Data lines to ADC
// D8-D10    Waveform       Control lines to ADC
// D11       Temperature    One-wire interface
// D12-D13   Serial         Software serial for tx/rx of commands
// A0        Voltage        To measure the recitified, divided voltage
// A1-A3     Capacitance    Uses an RC circuit to measure capacitance
// A4-A5     *unused*       *unused*


////////////////////////
//  #INCLUDES         //
////////////////////////
#include "math.h"            //needed to perform log() operation.
#include <OneWire.h>         //for the one-wire interface with the temperature sensor
#include <NewSoftSerial.h>




////////////////////////
//  Defined           //
//    Constants       //
////////////////////////
// -Serial code----------------
#define pin_RX 12             //pin 12 - RX for serial communication
#define pin_TX 13             //pin 13 - TX for serial communication
#define ERROR 'E'
#define LEFT 'L'
#define RIGHT 'R'
// -waveform code--------------
#define pin_RD 8              //pin 8 - connected to RD of the MAX153
#define pin_INT 9             //pin 9 - connected to INT of the MAX 153
#define pin_CS 10             //pin 10 - connected to CS of MAX 153
#define RD_low B11111110      //to set RD low, PORTB &= RD_low
#define RD_high  B00000001    //to set RD high, PORTB &= RD_high
#define INT_low B00000010     //if INT is low, PORTB & INT_low will = 0
#define Vsqmax 185            //maximum square wave voltage level
#define Vsqmin 47             //minimum square wave voltage level
#define num_samples 500       //number of samples
#define sq_thresh_percent .65 //x% of reading must be square to be a square wave
#define saw_thresh_percent .35
#define thresh_volt 0.15      //x% within the min or max = square, otherwise sawtooth

// -capRead code---------------
#define pin_CR1 A1            //samples the cap voltage (analog)
#define pin_CR2 A2            //discharges the capacitor (digital)
#define pin_CR3 A3            //charges the capacitor (digital)
#define MidCap 0.5            //uF ; above this is right, below is left
#define R 500.0               //Ohms ; Resistor that the cap charges through

// -voltRead code--------------
#define pin_volt A0
#define Voltmax 15            // The max voltage for the voltage task
#define Vdiode 0.57            // This is the turn on voltage for the diodes used.

// -temperature code-----------
#define pin_temp 11
const float TEMP_RANGE = 5.55555; // Optimal minimum range above or below room temp (5.55C=10F)
// Tolerance from optimal level (ex: temperature of +5C will trigger if TEMP_RANGE is 5.55C)
const float TRIGGER = 1.0;
// Room temperature amount to be used for now (will be removed)
//const float AMBIENT 23.0;
// Family ID of DS18B20 Temperature Sensor
const byte DS18B20_ID = 0x28;                                                             //
// Unique ID of plate sensor (8 bytes)                                                  ////
byte ID_AMBIENT[8] = {0x28, 0xD8, 0xF1, 0x87, 0x3, 0x0, 0x0, 0xAB};                  ////////////////----PUT TEMP SENSOR ROM ID'S HERE!!!!
// Unique ID of ambient sensor (8 bytes)                                                ////
byte ID_PLATE[8] = {0x28, 0xF0, 0x3E, 0x77, 0x3, 0x0, 0x0, 0x80};                         //
//byte ID_PLATE[8] = {0x28, 0xF9, 0x5F, 0x61, 0x3, 0x0, 0x0, 0xFB};


////////////////////////
//  List of Functions //  (because apparently Arduino doesn't need function prototypes?)
////////////////////////
//  void waveformSetup()  -  Prepares for reading the waveform.
//  char waveformRead() -  Reads the waveform and returns the status.
//  byte fullRead()       -  Used by waveformLoop to read and return Port D.
//  void capSetup()       -  Prepares for reading capacitance.
//  char capRead()      -  Reads capacitance and returns the status.
//  void voltSetup()      -  Prepares for reading voltage.
//  char voltRead()     -  Reads the voltage and returns the status.
//  void tempSetup()      -  Prepares for reading temperature.
//  char tempRead()     -  Reads the temperature and returns the status.
//  float readTemp(byte Addr[])  -  Used by tempRead() to get the temperature.


//Global variables/////
NewSoftSerial mySerial(pin_RX, pin_TX); //Set (Rx,Tx)
OneWire ds(pin_temp);


void setup()  
{
  // Setup Software serial for communicating with the other Arduino.
  // The other Arduino does not have to use NewSoftwareSerial; you can
  // use the regular Serial port (pins 1&2).  We had to use NewSoftwareSerial
  // here because the regular Serial pins have to be used for the ADC
  // for the waveform reader.
  mySerial.begin(9600); //Set baud tate
  mySerial.println("Hello, world?");
}

char char_last = 0;
boolean expectingFlag = false;
void loop()
{ 
  char char_rx = 0; //holder for the received character
  char char_tx = 0;
  if (mySerial.available())
  {
    char_rx = mySerial.read(); //read in character
    if (expectingFlag)
    {
      if (char_rx == 'K')
      {
        expectingFlag = false;
        return;
      }
      else
      {
        char_tx = char_last;
      }
      
    } // end if (expecting)
    else {
    switch (char_rx)
    {
      case '?':
        char_tx = 'K';
        break;
      case '1':
      case 'V':
        voltSetup();
        delay(1);
        char_tx = voltRead();
        expectingFlag = true;
        break;
      case '2':
      case 'T':
        tempSetup();
        delay(1);
        char_tx = tempRead();
        expectingFlag = true;
        break;
      case '3':
      case 'C':
        capSetup();
        delay(1);
        char_tx = capRead();
        expectingFlag = true;
        break;
      case '4':
      case 'W':
        waveformSetup();
        delay(1);
        char_tx = waveformRead();
        expectingFlag = true;
        break;
      default:
        char_tx = 'X';
    } // end switch
    } // end else (NOT expecting)
        
    if (char_tx != 0)    
      mySerial.print(char_tx);
    
  }// end if (available)

}


////////////////////////////////////////////////
//  waveformSetup                             //
////////////////////////////////////////////////
void waveformSetup()
{
 pinMode(pin_CS, OUTPUT); //Pin 12 will enable / disable the MAX 153
 pinMode(pin_RD, OUTPUT); //Pin 10 will be low to read in our values, and high when finished (RD)
 pinMode(pin_INT, INPUT);  //Pin 11 will read the ADC's interrupt output to see when to stop looping (INT)
 DDRD = 0;  //Sets Port D to be an input; will read D0 - D7
}


////////////////////////////////////////////////
//  waveformRead                              //
////////////////////////////////////////////////
char waveformRead()
{
  int sqcount = 0;
  int sawcount = 0; //count for the square wave/sawtooth wave, variable for signal value
  int my_status;
  byte val = 0;
  

  DDRD = 0;  //PORT D as input 
  digitalWrite(pin_CS,LOW); //enable ADC chip
  delay(1);  //give it time to start up
  for (int i=0; i<num_samples; i++)
  {
    unsigned int blah = 0;  //we should probably figure out how to get rid of this without it messing up
    val = 0;
    //digitalWrite(pin_RD, LOW);  //write RD LOW to read in data
    PORTB &= RD_low;  // replaces the commented line above
    //while(digitalRead(pin_INT)) {} //while INT is HIGH, the code waits
    while(PORTB & B00001000) {}  // replaces the commented line above
    delayMicroseconds(4);
    val = fullRead();  //read D0-D7
    blah = (unsigned int)val;
    //digitalWrite(pin_RD, HIGH); //stop reading in data
    PORTB |= RD_high;  // replaces the commented line above
    
    if ((blah*1.0 >= (Vsqmax - Vsqmax*thresh_volt)) && 
        (blah*1.0 <= (Vsqmax + Vsqmax*thresh_volt)))  //if voltage is within 10% of square wave maximum
      sqcount++;
    else if ((blah*1.0 >= (Vsqmin - Vsqmax*thresh_volt)) && 
             (blah*1.0 <= (Vsqmin + Vsqmax*thresh_volt)))  //if voltage is within 10% of square wave minimum
      sqcount++;
    else  //if voltage is neither, implying that it must be inbetween 
      sawcount++;
  }
  
  digitalWrite(pin_CS,HIGH);  //disables the ADC
    
  if (sqcount > num_samples*(sq_thresh_percent))   //if x% of samples are within range, then it must be a square wave
    return RIGHT;
  else if (sawcount > num_samples*(saw_thresh_percent))   //if x% of samples are NOT within range, then it must be a sawtooth wave
    return LEFT;
  else  //if for some reason neither fit
    return ERROR;
}


////////////////////////////////////////////////
//  fullRead                                  //
////////////////////////////////////////////////
byte fullRead()  //defines fullRead, the function that reads pins D0-D7 for input data
{
  byte result = PIND;
  return result;
}


////////////////////////////////////////////////
//  capSetup                                  //
////////////////////////////////////////////////
void capSetup()
{
  pinMode(pin_CR2,INPUT);
  pinMode(pin_CR3,INPUT);
  pinMode(pin_CR1,INPUT);
}


////////////////////////////////////////////////
//  capRead                                   //
////////////////////////////////////////////////
char capRead()
{
  //declare variables
  int V1, V2;  //holds voltage samples
  float C;     //holds the calculated capacitance (in uF)
  
  //setup
  pinMode(pin_CR2,OUTPUT);
  pinMode(pin_CR3,OUTPUT);
  digitalWrite(pin_CR1,LOW);
  pinMode(pin_CR1,INPUT);
  digitalWrite(pin_CR2,LOW);
  
  digitalWrite(pin_CR3,LOW); //discharge cap
  delay(1);  //make sure it's fully discharged
  pinMode(pin_CR3,INPUT); //stop discharging by setting this pin to a high impedance state
  delayMicroseconds(4);
  digitalWrite(pin_CR2,HIGH); //start charging
  V1 = analogRead(pin_CR1); // get first reading
  V2 = analogRead(pin_CR1); // occurs 112us after first reading
  
  C = 112.0 / (R * log(float(1023-V1)/float(1023-V2)));
  // ^ fancy mathematics
  
  if (V1 >= 900)      //bad connection
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


////////////////////////////////////////////////
//  voltSetup                                 //
////////////////////////////////////////////////
void voltSetup()
{
  pinMode(pin_volt,INPUT);
}


////////////////////////////////////////////////
//  voltRead                                  //
////////////////////////////////////////////////
char voltRead()
{
  int Vin;
  double vActual;
  Vin = analogRead(pin_volt);
  vActual = ((3*5*Vin)/1023) + 2*Vdiode; //find and put Vactual in fullscale voltage
  if (vActual > 10 && vActual <= 15)  //11V to 15V = turn right
  { return RIGHT; }
  else if (vActual >= 4.5)  //5V to 9V = turn left
  { return LEFT; }
  else
  { return ERROR; }  // less than 5 (with a little margin of error) indicates error.
}


 
////////////////////////////////////////////////
//  tempSetup                                 //
////////////////////////////////////////////////
void tempSetup()
{
  
}
     
////////////////////////////////////////////////
//  tempRead                                  //
////////////////////////////////////////////////
char tempRead() {

  float tempPlate, tempAmbient;
  
  ds.reset();
  ds.skip();
  ds.write(0x44);
  
  delay(1000);            // Measurement may take 750ms
  
  tempPlate = readTemp(ID_PLATE);
  delay(1000);
  tempAmbient = readTemp(ID_AMBIENT);
  
  if (tempPlate > tempAmbient + TEMP_RANGE - TRIGGER)
  {
    return RIGHT;
  }
  else if (tempPlate < tempAmbient - TEMP_RANGE + TRIGGER)
  {
    return LEFT;
  }
  else
  {
    return ERROR;
  }
}

////////////////////////////////////////////////
//  readTemp                                  //
////////////////////////////////////////////////
float readTemp(byte addr[]) {  //not to be confused with tempRead()
  
  int HighByte, LowByte, TReading;
  float Tc_100;
  byte i;
  byte data[2];
  
  ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read scratchpad command
 
  for ( i = 0; i < 2; i++) {     // Read 9 bytes of data
    data[i] = ds.read();
  }
  
  LowByte = data[0];
  HighByte = data[1];
  TReading = (HighByte << 8) + LowByte;       // Raw temperature from sensor
  Tc_100 = (6 * TReading) + TReading / 4.;    // Multiply by 6.25 to get degCelsius
 
  return Tc_100; 
}
