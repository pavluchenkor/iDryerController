// 3950 THERMISTOR EXAMPLE.
// Written by Miguel Angel Califa Urquiza
// Released under an MIT license.

// Depends on the following Arduino libraries:
// - Arduino thermistor library: https://github.com/miguel5612/Arduino-ThermistorLibrary

#include <thermistor.h>

thermistor therm1(A0,0);  // Analog Pin which is connected to the 3950 temperature sensor, and 0 represents TEMP_SENSOR_0 (see configuration.h for more information).


void setup() {
  // put your setup code here, to run once: 
  Serial.begin(9600); //initialize port serial at 9600 Bauds.
}

void loop() {
  double temp = therm1.analog2tempEADC(1024); // read temperature based in external 1024 example value
  //Print temperature in port serial
  Serial.print("Temperatura: "); 
  Serial.println((String)temp);
  Serial.print("----------------------");
  delay(2000); //wait 2000 mS for next measure
}