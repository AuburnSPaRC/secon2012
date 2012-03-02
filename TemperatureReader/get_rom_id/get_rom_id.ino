#include <OneWire.h>

  #define RELAY_K1_PIN   52 // (DIG) DPin 52 - Control Relay 1
  #define RELAY_K2_PIN   53 // (DIG) DPin 53 - Control Relay 

// OneWire device bus
OneWire db(A11);  // on pin 10

void setup(void) {
  // Initialize inputs/outputs
  // Start serial port
  Serial.begin(9600);
  
  digitalWrite(RELAY_K1_PIN, 0);
  digitalWrite(RELAY_K2_PIN, 1);
  // -------------  
  delay(100);
  digitalWrite(RELAY_K1_PIN, 1);
  digitalWrite(RELAY_K2_PIN, 0);
  // -------------  
  delay(100);  

  byte i;
  byte present = 0;
  byte addr[8];

  delay(4000);
  
  while ( db.search(addr)) {

    Serial.print("ADDR = {0x");

    for( i = 0; i < 7; i++) {
      Serial.print(addr[i], HEX);
      Serial.print(", 0x");
    }
    Serial.print(addr[7], HEX);
    Serial.print("}\n");

    if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
    }

    if (addr[0] == 0x10) {
      Serial.println("Device is a DS18S20 family device: 0x10");
      Serial.println("Digital Thermometer Device");
      Serial.println("---------------------------------");
    }
    else if (addr[0] == 0x28) {
      Serial.println("Device is a DS18S20 family device: 0x28");
      Serial.println("Digital Thermometer Device");
      Serial.println("---------------------------------");  
    }
    else if (addr[0] == 0x01) {
      Serial.println("Device is a DS2401 family device: 0x01");
      Serial.println("Digital Serial Number Device");
      Serial.println("---------------------------------");  
    }
    else {
      Serial.println("Device family is not recognized: 0x");
      Serial.println(addr[0],HEX);

    }
  }
  
  Serial.print("No more addresses.\n");
  db.reset_search();

  return;
}

void loop(void) {

}




