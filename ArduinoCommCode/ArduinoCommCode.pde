
#include <NewSoftSerial.h>

//This program prints/reads characters from the keyboard into one
//Serial port, subtracts 1, transmits it over mySerial, subtracts
//1 again, and reads/prints it back through another arduino's
//serial port.

NewSoftSerial mySerial(2, 3); //Set (Rx,Tx)

void setup()  
{
  Serial.begin(9600); //Set Baud rate
  Serial.println("Goodnight moon!");

  mySerial.begin(9600); //Set baud tate
  mySerial.println("Hello, world?");
}

void loop()
{
  if (mySerial.available()) { //Check if buffer is clear
      Serial.print((char)(mySerial.read()-1)); //Print char in buffer-1
  }
  if (Serial.available()) { //Check if buffer is clear
      mySerial.print((char)(Serial.read()-1)); //Print char in buffer-1
  }
}
