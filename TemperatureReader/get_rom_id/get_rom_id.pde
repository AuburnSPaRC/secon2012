    #include <OneWire.h>
     
    // OneWire device bus
    OneWire db(10);  // on pin 10
     
    void setup(void) {
      // Initialize inputs/outputs
      // Start serial port
      Serial.begin(9600);
      
      byte i;
      byte present = 0;
      byte addr[8];
     
      if ( !db.search(addr)) {
          Serial.print("No more addresses.\n");
          db.reset_search();
          return;
      }
     
      Serial.print("ADDR = {0x");
	  
      for( i = 0; i < 7; i++) {
        Serial.print(addr[i], HEX);
		Serial.print(", 0x");
      }
		Serial.print(addr[7], HEX);
		Serial.print("}");
     
      if ( OneWire::crc8( addr, 7) != addr[7]) {
          Serial.print("CRC is not valid!\n");
          return;
      }
     
      if ( addr[0] == 0x10) {
          Serial.print("Device is a DS18S20 family device: 0x10\n");
      }
      else if ( addr[0] == 0x28) {
          Serial.print("Device is a DS18B20 family device: 0x28\n");
      }
      else {
          Serial.print("Device family is not recognized: 0x");
          Serial.println(addr[0],HEX);
          return;
      }
     
    }
     
    void loop(void) {
     
    }
