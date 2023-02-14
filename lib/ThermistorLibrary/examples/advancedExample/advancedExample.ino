// 3950 THERMISTOR EXAMPLE.
// Written by Miguel Angel Califa Urquiza
// Released under an MIT license.

// Depends on the following Arduino libraries:
// - Arduino thermistor library: https://github.com/miguel5612/Arduino-ThermistorLibrary

// At this example we use two diferent thermistor, one is 3950 thermistor and two is unknown thermistor.
// you can configurate this thermistor in Configuration.h in Documents/arduino/libraries/Arduino-ThermistorLibrary/src/Configuration.h and select TEMP_SENSOR_1 value. 
#include <thermistor.h>

thermistor therm1(A0,0);  // Analog Pin which is connected to the 3950 temperature sensor, and 0 represents TEMP_SENSOR_0 (see configuration.h for more information).
thermistor therm2(A1,1);  // Analog Pin which is connected  temperature sensor, and 0 represents TEMP_SENSOR_1 (see configuration.h for more information).


void setup() {
  // put your setup code here, to run once: 
  Serial.begin(9600); //initialize port serial at 9600 Bauds.
}

void loop() {
  // All thermistor temperature is in degrade.
  double temp1 = therm1.analog2temp(); // read temperature
  double temp2 = therm2.analog2temp(); // read temperature
  //Print temperature in port serial
  Serial.print("Temperatura del termistor 1: "); 
  Serial.println((String)temp1);
  Serial.print("----------------------");
  
  Serial.print("Temperatura del termistor 2: "); 
  Serial.println((String)temp2);
  Serial.print("----------------------");
  delay(2000); //wait 2000 mS for next measure
}
