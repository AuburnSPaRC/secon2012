#include <OneWire.h>

// -- CONSTANTS --
// Optimal minimum range above or below room temp (5.55C=10F)
const float TEMP_RANGE = 5.55555; 
// Tolerance from optimal level (ex: temperature of +5C will trigger if TEMP_RANGE is 5.55C)
const float TRIGGER = 1.0;
// Room temperature amount to be used for now (will be removed)
//const float AMBIENT 23.0;
// Family ID of DS18B20 Temperature Sensor
const byte DS18B20_ID = 0x28;
// Unique ID of plate sensor (8 bytes)
byte ID_PLATE[8] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// Unique ID of ambient sensor (8 bytes)
byte ID_AMBIENT[8] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// -- I/O PIN-MAPPING --

// DS18S20 Temperature chip i/o
OneWire ds(10);  // on pin 10

// -- SETUP FUNCTION --
void setup(void) {

  Serial.begin(9600);    // Start serial port

}

// -- MAIN LOOP --
void loop(void) {

  float tempPlate, tempAmbient;

  ds.reset();
  ds.select(ID_PLATE);
  ds.write(0x44);         // Start measurement command

  ds.reset();
  ds.select(ID_AMBIENT);
  ds.write(0x44);         // Start measurement command

  delay(1000);            // Measurement may take 750ms

  tempPlate = readTemp(ID_PLATE);
  tempAmbient = readTemp(ID_AMBIENT);

  if (tempPlate > tempAmbient + TEMP_RANGE - TRIGGER)
  {
    Serial.print("RIGHT");
  }
  else if (tempPlate < tempAmbient - TEMP_RANGE + TRIGGER)
  {
    Serial.print("LEFT");
  }
  else
  {
    Serial.print("ERROR");
  }

  Serial.print("\n");
}

// -- DATA READ AND CONVERSION --
float readTemp(byte addr[]) {

  int msByte, lsByte, tempReading;
  float tempCel;


  ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read-scratchpad command

  lsByte = ds.read();     // Get the least-sig byte first
  msByte = ds.read();     // Then the most-sig byte

  tempReading = (msByte << 8) + lsByte;  // Raw temperature from sensor
  tempCel = 0.0625 * tempReading;        // Multiply by 6.25/1000 to get degCelsius

  return tempCel; 
}

