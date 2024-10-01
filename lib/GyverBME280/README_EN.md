This is an automatic translation, may be incorrect in some places. See sources and examples!

# Gyverbme280
Light library for working with BME280 on I2C for Arduino

## compatibility
Compatible with all arduino platforms (used arduino functions)

## Content
- [installation] (# Install)
- [initialization] (#init)
- [use] (#usage)
- [Example] (# Example)
- [versions] (#varsions)
- [bugs and feedback] (#fedback)

<a id="install"> </a>
## Installation
- The library can be found by the name ** gyverbme280 ** and installed through the library manager in:
    - Arduino ide
    - Arduino ide v2
    - Platformio
- [download library] (https://github.com/gyverlibs/gyverbme280/archive/refs/heads/main.zip). Zip archive for manual installation:
    - unpack and put in * C: \ Program Files (X86) \ Arduino \ Libraries * (Windows X64)
    - unpack and put in * C: \ Program Files \ Arduino \ Libraries * (Windows X32)
    - unpack and put in *documents/arduino/libraries/ *
    - (Arduino id) Automatic installation from. Zip: * sketch/connect the library/add .Zip library ... * and specify downloaded archive
- Read more detailed instructions for installing libraries [here] (https://alexgyver.ru/arduino-first/#%D0%A3%D1%81%D1%82%D0%B0%BD%D0%BE%BE%BE%BED0%B2%D0%BA%D0%B0_%D0%B1%D0%B8%D0%B1%D0%BB%D0%B8%D0%BE%D1%82%D0%B5%D0%BA)
### Update
- I recommend always updating the library: errors and bugs are corrected in the new versions, as well as optimization and new features are added
- through the IDE library manager: find the library how to install and click "update"
- Manually: ** remove the folder with the old version **, and then put a new one in its place.“Replacement” cannot be done: sometimes in new versions, files that remain when replacing are deleted and can lead to errors!


<a id="init"> </a>
## initialization
`` `CPP
Gyverbme280 BME;
`` `

<a id="usage"> </a>
## Usage
`` `CPP
Bool Begin (VOID);// Launch with a standard address (0x76)
Bool Begin (Uint8_T Address);// Launch addressing the address
Bool ISMEASURING (VOID);// returns True while the measurement is underway
Float Readpressure (Void);// Read the pressure in PA
Float Readhumidity (VOID);// read humidity in %
VOID OneMeasurement (VOID);// make one dimension and go into a dream
Float ReadTemperature (VOID);// Read the temperature in degrees with

VOID setmode (uint8_t mode);// set the operating mode
// modes:
Normal_Mode
Forced_Mode

VOID SetFilter (Uint8_t Mode);// Change the filtration coefficient.Call before Begin
// coefficients:
Filter_disable
Filter_coEF_2
Filter_coEF_4
Filter_coEF_8
Filter_coEF_16

VOID SetstandbyTime (Uint8_t Mode);// Change the time between measurements.Call before Begin
// modes:
Standby_500us
Standby_10ms
Standby_20ms
Standby_6250us
Standby_125MS
Standby_250MS
Standby_500ms
Standby_1000ms

VOID sethumoversampling (Uint8_t Mode);// Configure oversemling or turn off humidity.Call before Begin
VOID settempoVersampling (Uint8_t Mode);// Configure oversemling or turn off the temperature.Call before Begin
VOID SetPressoversampling (Uint8_t Mode);// configure oversemling or disconnect dCranberry.Call before Begin
// modes:
Module_disable
Oversampling_1
Oversampling_2
Oversampling_4
Oversampling_8
Oversampling_16
`` `

<a id="EXAMPLE"> </a>
## Example
The rest of the examples look at ** Examples **!
`` `CPP
/*
   A simple example, demonstrating the basic functions of measuring temperature, pressure and humidity
*/

#incLude <gyverbme280.h> // Library connection
Gyverbme280 BME;// Creating a BME Object

VOID setup () {
  Serial.Begin (9600);// launch of the sequential port
  bme.begin ();// if additional.Settings are not needed - initialize the sensor
}

VOID loop () {
  Serial.print ("Temperature:");
  Serial.print (BME.Readtemperature ());// We display the temporary core in [*c]
  Serial.println (" *c");

  Serial.print ("Humidity:");
  Serial.print (bme.readhumidity ());// We display humidity in [%]
  Serial.println (" %");

  Float Pressure = Bme.Readpressure ();// read pressure in [pa]
  Serial.print ("Pressure:");
  Serial.print (Pressure / 100.0F);// Removing pressure in [GPA]
  Serial.print ("hpa,");
  Serial.print (Pressuretommhg (Pressure));// Removing the pressure in [mm RT.pillar]
  Serial.println ("MM HG");
  Serial.print ("altitide:");
  Serial.print// We display height B [m above the ur.seas]
  Serial.println ("m");
  Serial.println ("");
  DELAY (1000);
}
`` `

<a id="versions"> </a>
## versions
- V1.3 - Fixed error when ledge.temperature
- v1.4 - divided into h and CPP
- V1.5 - Added BMP280 support

<a id="feedback"> </a>
## bugs and feedback
Create ** Issue ** when you find the bugs, and better immediately write to the mail [alex@alexgyver.ru] (mailto: alex@alexgyver.ru)
The library is open for refinement and your ** pull Request ** 'ow!


When reporting about bugs or incorrect work of the library, it is necessary to indicate:
- The version of the library
- What is MK used
- SDK version (for ESP)
- version of Arduino ide
- whether the built -in examples work correctly, in which the functions and designs are used, leading to a bug in your code
- what code has been loaded, what work was expected from it and how it works in reality
- Ideally, attach the minimum code in which the bug is observed.Not a canvas of a thousand lines, but a minimum code