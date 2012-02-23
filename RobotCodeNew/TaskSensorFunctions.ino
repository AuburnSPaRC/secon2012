/**  Code for controlling the sensors that read the four task boxes.
*
* This code is for the controlling the sensors that read the four task boxes: voltage, 
* temperature, waveform, and capcitance. For a schematic of the circuit this code is 
* intended for, check redmine.
*
* Depends: math.h
*          OneWire.h
*
* Credits: Ben Straub, Christopher James, Tyler Crumpton, Kayla Frost, and Ray Preston.
*
*/

#include "math.h"      // Needed to perform log() operation.
#include <OneWire.h>   // For the one-wire interface with the temperature sensor.

// --- Sensor Pin Definitions ---
// NOTE: Defaults only; please edit the values in PinDefines.h!
#ifndef OVERWRITE_SENSOR_PINS
  // Waveform Pins (D0-D7 also used, PORTB):
  #define PIN_RD   8   // (DIG) DPin 8  - Connected to RD of the MAX153
  #define PIN_INT  9   // (DIG) DPin 9  - Connected to INT of the MAX 153
  #define PIN_CS   10  // (DIG) DPin 10 - Connected to CS of MAX 153
  // Capcitance Pins:
  #define PIN_CR1  A1  // (ANA) APin 1  - Samples the cap voltage (analog)
  #define PIN_CR2  A2  // (DIG) APin 2  - Discharges the capacitor (digital)
  #define PIN_CR3  A3  // (DIG) APin 3  - Charges the capacitor (digital)
  // Voltage Pins:
  #define PIN_VOLT A0  // (ANA) APin0   - Measure voltage
  // Temperature Pins:
  #define PIN_TEMP 11  // (DIG) DPin11  - For measuring temperature
#endif
  
#define ADC_PORT PORTA
#define ADC_PIN PINA
#define ADC_DDR DDRA
#define ADC_RD_PORT PORTC
#define ADC_RD_PIN PINC
#define ADC_RD_SET_MASK 0b10000000
#define ADC_RD_CLR_MASK 0b01111111
#define ADC_INT_PORT PORTC
#define ADC_INT_PIN PINC
#define ADC_INT_SET_MASK 0b00100000
#define ADC_INT_CLR_MASK 0b11011111

// --- Constants ---
#define LEFT  true             // Left direction
#define RIGHT false            // Right direction
#define ERROR LEFT             // Default direction when sensor reading fails

// Waveform Constants:
#define V_SQ_MAX    185        // Maximum square wave voltage level
#define V_SQ_MIN    47         // Minimum square wave voltage level
#define NUM_SAMPLES 500        // Number of samples
#define VOLT_THRESH 0.15       // Voltage threshold
#define SQ_THRESH_PERCENT  .65 // X% of reading must be square to be a square wave
#define SAW_THRESH_PERCENT .35 // X% within the min or max = square, otherwise sawtooth

// Capcitor Constants:
#define MidCap      0.5        // uF ; above this is right, below is left
#define R           470.0      // Ohms ; Resistor that the cap charges through

// Voltage Constants:
#define Voltmax     15         // The max voltage for the voltage task
#define Vdiode      0.57       // This is the turn on voltage for the diodes used.

// Temperature Constants:
#define TEMP_RANGE  5.55       // Optimal minimum range above or below room temp (5.55C=10F)
#define TRIGGER     1.0        // Tolerance from TEMP_RANGE level 
#define DS18B20_ID  0x28       // Family ID of DS18B20 Temperature Sensor
byte ID_AMBIENT[8] = {0x28, 0xD8, 0xF1, 0x87, 0x3, 0x0, 0x0, 0xAB}; // ID of ambient sensor.
byte ID_PLATE[8] = {0x28, 0xF0, 0x3E, 0x77, 0x3, 0x0, 0x0, 0x80};   // ID of plate sensor.


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


// Global variables
OneWire ds(PIN_TEMP);
boolean accurateFlag;  // Is the turn decision accurate? (true for 'yes')

boolean readWaveform()
{
  // --- Setup ---
  pinMode(PIN_CS, OUTPUT); //Pin 12 will enable / disable the MAX 153
  pinMode(PIN_RD, OUTPUT); //Pin 10 will be low to read in our values, and high when finished (RD)
  pinMode(PIN_INT, INPUT);  //Pin 11 will read the ADC's interrupt output to see when to stop looping (INT)
  ADC_DDR = 0;  //Sets ADC Port to be an input
  digitalWrite(RELAY_K1_PIN, 0);
  digitalWrite(RELAY_K2_PIN, 0);
  // -------------

  int sqcount = 0;
  int sawcount = 0; //count for the square wave/sawtooth wave, variable for signal value
  int my_status;
  byte val = 0;
  
  ADC_DDR = 0;  //ADC_PORT as input
  digitalWrite(PIN_CS,LOW); //enable ADC chip
  delay(1);  //give it time to start up
  for (int i=0; i<NUM_SAMPLES; i++)
  {
    unsigned int blah = 0;  //we should probably figure out how to get rid of this without it messing up
    val = 0;
    //digitalWrite(pin_RD, LOW);  //write RD LOW to read in data
    ADC_RD_PORT &= ADC_RD_CLR_MASK;  // replaces the commented line above
    //while(digitalRead(pin_INT)) {} //while INT is HIGH, the code waits
    while(ADC_INT_PORT & ADC_INT_SET_MASK) {}  // replaces the commented line above
    delayMicroseconds(4);
    val = ADC_PIN;  //read adc pins
    blah = (unsigned int)val;  //today's non-descriptive variable name is brought to you by Ben Straub
    //digitalWrite(pin_RD, HIGH); //stop reading in data
    ADC_RD_PORT |= ADC_RD_SET_MASK;  // replaces the commented line above
    
    if ((blah*1.0 >= (V_SQ_MAX - V_SQ_MAX*VOLT_THRESH)) && 
        (blah*1.0 <= (V_SQ_MAX + V_SQ_MAX*VOLT_THRESH)))  //if voltage is within 10% of square wave maximum
      sqcount++;
    else if ((blah*1.0 >= (V_SQ_MIN - V_SQ_MAX*VOLT_THRESH)) && 
             (blah*1.0 <= (V_SQ_MIN + V_SQ_MAX*VOLT_THRESH)))  //if voltage is within 10% of square wave minimum
      sqcount++;
    else  //if voltage is neither, implying that it must be inbetween 
      sawcount++;
  }
  
  digitalWrite(PIN_CS,HIGH);  //disables the ADC
    
  if (sqcount > NUM_SAMPLES*(SQ_THRESH_PERCENT))   //if x% of samples are within range, then it must be a square wave
  {
    accurateFlag = true;
    return RIGHT;
  }
  else if (sawcount > NUM_SAMPLES*(SAW_THRESH_PERCENT))   //if x% of samples are NOT within range, then it must be a sawtooth wave
  {
    accurateFlag = true;
    return LEFT;
  }
  else
  {
    accurateFlag = false;
    return ERROR;  
  }
}

boolean readCapacitance()
{
  // --- Setup ---
  pinMode(PIN_CR2,INPUT);
  pinMode(PIN_CR3,INPUT);
  pinMode(PIN_CR1,INPUT);
  digitalWrite(RELAY_K1_PIN, 1);
  digitalWrite(RELAY_K2_PIN, 0);
  // -------------
  
    //declare variables
  int V1, V2;  //holds voltage samples
  float C;     //holds the calculated capacitance (in uF)
  
  //setup
  pinMode(PIN_CR2,OUTPUT);
  pinMode(PIN_CR3,OUTPUT);
  digitalWrite(PIN_CR1,LOW);
  pinMode(PIN_CR1,INPUT);
  digitalWrite(PIN_CR2,LOW);
  
  digitalWrite(PIN_CR3,LOW); //discharge cap
  delay(1);  //make sure it's fully discharged
  pinMode(PIN_CR3,INPUT); //stop discharging by setting this pin to a high impedance state
  delayMicroseconds(4);
  digitalWrite(PIN_CR2,HIGH); //start charging
  V1 = analogRead(PIN_CR1); // get first reading
  V2 = analogRead(PIN_CR1); // occurs 112us after first reading
  
  C = 112.0 / (R * log(float(1023-V1)/float(1023-V2)));
  // ^ fancy mathematics
   Serial.print("voltage");
  Serial.print(V1);
  if (V1 >= 900)      //bad connection
  {
    accurateFlag = false;
    return ERROR;
  }
  else if (V2 == 1023) //cap is too small
  {
    accurateFlag = true;
    return LEFT;
  }
  else if (V1 == V2)  //cap is too big
  {
    accurateFlag = true;
    return RIGHT;
  }
  else if (C > MidCap)
  {
    accurateFlag = true;
    return RIGHT;
  }
  else
  {
    accurateFlag = true;
    return LEFT;
  }
}

boolean readVoltage()
{
  // --- Setup ---
  pinMode(PIN_VOLT,INPUT);
  digitalWrite(RELAY_K1_PIN, 0);
  digitalWrite(RELAY_K2_PIN, 1);
  // -------------  
  
  int Vin;
  double vActual;
  Vin = analogRead(PIN_VOLT);
  vActual = ((3*5*Vin)/1023) + 2*Vdiode; //find and put Vactual in fullscale voltage
  
  Serial.print("voltage");
  Serial.print(vActual);
  if (vActual > 10 && vActual <= 15)  //11V to 15V = turn right
  {
    accurateFlag = true;
    return RIGHT; 
  }
  else if (vActual >= 4.5)  //5V to 9V = turn left
  {
    accurateFlag = true;
    return LEFT;
  }
  else
  {
    accurateFlag = false;
    return ERROR;  // less than 5 (with a little margin of error) indicates error.  
  }
}
     
boolean readTemperature() 
{
  float tempPlate, tempAmbient;
  
  digitalWrite(RELAY_K1_PIN, 0);
  digitalWrite(RELAY_K2_PIN, 0);
  
  ds.reset();
  ds.skip();
  ds.write(0x44);
  
  delay(1000);            // Measurement may take 750ms
  
  tempPlate = readSensorTemp(ID_PLATE);
  delay(1000);
  tempAmbient = readSensorTemp(ID_AMBIENT);
  
  if (tempPlate > tempAmbient + TEMP_RANGE - TRIGGER)
  {
    accurateFlag = true;
    return RIGHT;
  }
  else if (tempPlate < tempAmbient - TEMP_RANGE + TRIGGER)
  {
    accurateFlag = true;
    return LEFT;
  }
  else
  {
    accurateFlag = false;
    return ERROR;
  }
}

float readSensorTemp(byte addr[]) 
{   
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

boolean getAccurateFlag()
{
  return accurateFlag;  
}
