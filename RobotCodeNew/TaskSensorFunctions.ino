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
byte ID_AMBIENT[8] = {0x28, 0x3C, 0x2F, 0xBB, 0x3, 0x0, 0x0, 0xF8}; // ID of ambient sensor.
byte ID_PLATE[8] = {0x28, 0xF9, 0x48, 0xBB, 0x3, 0x0, 0x0, 0xCC};   // ID of plate sensor.


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

  delay(100);
  int sqcount = 0;
  int sawcount = 0; //count for the square wave/sawtooth wave, variable for signal value
  int my_status;
  byte val = 0;
  
  #ifdef DEBUG_WAVEFORM
    Serial.println("WAVEFORM----");
  #endif
  
  
  digitalWrite(PIN_CS,LOW); //enable ADC chip
  delay(1);  //give it time to start up
  for (int i=0; i<NUM_SAMPLES; i++)
  {
    unsigned int blah = 0;  //we should probably figure out how to get rid of this without it messing up
    val = 0;
    //digitalWrite(pin_RD, LOW);  //write RD LOW to read in data
    ADC_RD_PORT &= ADC_RD_CLR_MASK;  // replaces the commented line above
    //while(digitalRead(pin_INT)) {} //while INT is HIGH, the code waits
    while(ADC_INT_PIN & ADC_INT_SET_MASK) {}  // replaces the commented line above
    delayMicroseconds(4);
    val = ADC_PIN;  //read adc pins
    blah = (unsigned int)val;  //today's non-descriptive variable name is brought to you by Ben Straub
    
    //digitalWrite(pin_RD, HIGH); //stop reading in data
    ADC_RD_PORT |= ADC_RD_SET_MASK;  // replaces the commented line above
    
    #ifdef DEBUG_WAVEFORM
      Serial.println(blah);
    #endif
    
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
  
    Serial.print("Square: ");
  Serial.println(sqcount);
  Serial.print("Saw: ");
  Serial.println(sawcount);
//  Serial.print("time: ");
//  Serial.println(t2-t1);
  Serial.println("-------------");
  
  if (sqcount > NUM_SAMPLES*(SQ_THRESH_PERCENT))   //if x% of samples are within range, then it must be a square wave
  {
    accurateFlag = true;
    Serial.print("RIGHT");    
    return RIGHT;
  }
  else if (sawcount > NUM_SAMPLES*(SAW_THRESH_PERCENT))   //if x% of samples are NOT within range, then it must be a sawtooth wave
  {
    accurateFlag = true;
    Serial.print("LEFT");
    return LEFT;
  }
  else
  {
    accurateFlag = false;
    Serial.print("ERROR");
    return ERROR;  
  }
}

boolean readCapacitance()
{
  // --- Setup ---
  pinMode(PIN_CR2,INPUT);
  pinMode(PIN_CR3,INPUT);
  pinMode(PIN_CR1,INPUT);
  digitalWrite(RELAY_K1_PIN, 0);
  digitalWrite(RELAY_K2_PIN, 1);
  // -------------
  delay(100);
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
  delay(4);  //make sure it's fully discharged
  pinMode(PIN_CR3,INPUT); //stop discharging by setting this pin to a high impedance state
  delayMicroseconds(4);
  digitalWrite(PIN_CR2,HIGH); //start charging
  V1 = analogRead(PIN_CR1); // get first reading
  V2 = analogRead(PIN_CR1); // occurs 112us after first reading
  
  C = float(112.0 / (R * log(float(1023-float(V1))/float(1023-float(V2)))));
  // ^ fancy mathematics
   Serial.print("Capacitance:\n");
   Serial.println(V1);
   Serial.println(V2);
  Serial.println(C,5);
  Serial.print("\n");
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
  digitalWrite(RELAY_K1_PIN, 1);
  digitalWrite(RELAY_K2_PIN, 0);
  // -------------  
  delay(100);
  int Vin;
  double vActual;
  Vin = analogRead(PIN_VOLT);
  vActual = float(((3.0*5.0*float(Vin))/1023.0) + 2*Vdiode); //find and put Vactual in fullscale voltage
  
  Serial.print("Voltage: ");
  Serial.print(vActual);  Serial.print("\n");
  if (vActual > 10 && vActual <= 20)  //11V to 15V = turn right
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
  int i, numTimes=0, upping=0;   //Trending variables  0=down, 1=up
  static float lastTempPlate;
  digitalWrite(RELAY_K1_PIN, 1);
  digitalWrite(RELAY_K2_PIN, 1);
 
  delay(100);
  
  ds.reset();
  ds.skip();
  ds.write(0x44);
  

  tempAmbient = readSensorTemp(ID_AMBIENT);
  
  for(i=0;i<30;i++)    //Wait at maximum, ten readings
  {
    ds.reset();
    ds.skip();
    ds.write(0x44);
    delay(300);
    tempPlate = readSensorTemp(ID_PLATE); 
    tempAmbient=tempPlate;///REMOVE THIS LATER!*/
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
      if(upping)
      {
        if(tempPlate>lastTempPlate)
        {
          numTimes++;
        }
        else if(tempPlate<lastTempPlate)
        {
          numTimes=0;
          upping=false;
        }
      }
      else
      {
        if(tempPlate<lastTempPlate)
        {
          numTimes++;
        }
        else if(tempPlate>lastTempPlate)
        {
          numTimes=0;
          upping=true;
        }
      }
      if(numTimes>=4)
      {
        if(!upping){Serial.print("LEFT");return LEFT;}
        else {Serial.print("RIGHT");return RIGHT;}        
      }
    }
    lastTempPlate=tempPlate;
  }
  
    
  accurateFlag = false;
  if(tempPlate<tempAmbient){return LEFT;}
  else if(tempPlate>tempAmbient){return RIGHT;}
  Serial.print("ERROR");return ERROR;
  
  
}

float readSensorTemp(byte addr[]) 
{   
  int HighByte, LowByte, TReading;
  float Tc_100;
  byte i;
  byte data[2];
  int whole, fract;
  
  ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read scratchpad command
 
  for ( i = 0; i < 2; i++) {     // Read 9 bytes of data
    data[i] = ds.read();
  }
  
  LowByte = (int)data[0];  
  HighByte = (int)data[1];

  TReading = (HighByte << 8)+LowByte;       // Raw temperature from sensor
  Tc_100 = float((6.0 * float(TReading)) + float(TReading)/4.0);    // Multiply by 6.25 to get degCelsius
  whole=Tc_100/100;
  fract=int(Tc_100) % 100;
  Tc_100=float(whole+float(float(fract)/100));
  /*Serial.print(whole);
  Serial.print(".");
  if(fract<10)Serial.print("0");
  Serial.print(fract);
  Serial.print("\n");*/
  
  Serial.println(Tc_100);
  return Tc_100; 
}

boolean getAccurateFlag()
{
  return accurateFlag;  
}
