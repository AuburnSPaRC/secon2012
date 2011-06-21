// author: Ray Preston
// date:   6/6/2011
// updated: 6/11/2011
// Calculates the voltage input using the following
// current divider. Place arduino across V and unknown
// voltage accross A and B.
//
//                 ,----------o A
//                 |
//      ,----|>|-------|>|----.
//      |                     |
//      |    1K Ohm   2K Ohm  |
//  o---|---o-^^^-o-----^^^---|
// GND  |   -  V  +           |
//      `----|>|--------|>|---/
//                 |
//                 `----------o B
// This is designed for an unkown voltage source between
// zero and 15 volts. Resistors should keep the same voltage drop
// ratio to avoid damaging the arduino. *Max error of 0.9% under actual.

#define Vmax 15
#define Vt 0.7 // This is the turn on voltage for the diodes used.
               // Was originally assumed to be 1 until actually
               // diodes are chosen.
#include "math.h"

int Vin;
double vActual;

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  Vin = analogRead(A0);
  vActual = (Vin - (2 * Vt)) * (Vmax/1023.0) + (2 * Vt);  // put find Vin in fullscale voltage
  Serial.print("Voltage was ");
  Serial.println(vActual); // print out the fullscale voltage
  delay(500);
}
