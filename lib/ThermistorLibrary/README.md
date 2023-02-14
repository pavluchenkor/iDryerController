<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
[![DOI](https://zenodo.org/badge/170540207.svg)](https://zenodo.org/badge/latestdoi/170540207)
![Build Status][build-url]
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]

# Arduino thermistor library

### Need help or have a question? Send me an [email](mailto:miguelangelcu@ufps.edu.co), [email2](mailto:miguelangel5612@gmail.com)

## Contents
- [Development Status](#development-status)
- [Installation Instructions](#installation-instructions)
- [Issue/Bug report template](#issuebug-report-template)
- [Supported devices](#supported-thermistors)
- [Temperature sensors available](#temperature-sensors-available)
- [Acknowledgements](#acknowledgements)
- [More information](#more-information)
## Development Status
Most marlin is applied to calculate the temperature accurately.
- Supports any number of thermistors, the only thing that limits you is the number of analog inputs your arduino card has
- The temperature output is in degrees centigrade.

## Installation Instructions

- Using Arduino IDE
  + Clic in green Clone or download button.
  + Select download as zip.	
  + Follow this instruccions: [Instructions for Windows or mac](https://www.arduino.cc/en/Guide/Libraries)

#### Issue/Bug report template

Before reporting an issue, make sure you've searched for similar one that was already created. 

## Example with ramps schematic

![Pin Functions](http://chrisbarrbuilds.com/wp-content/uploads/2015/07/thermistorDiagram.png)

## Supported thermistors

All definitions in src/Configuration.h file.

Example configuration

*  #define TEMP_SENSOR_0 80 // This is thermistor 0
*  #define TEMP_SENSOR_1 0 // This is thermistor 1
*  #define TEMP_SENSOR_2 0 // This is thermistor 2
*  #define TEMP_SENSOR_3 0 // This is thermistor 3
*  #define TEMP_SENSOR_4 0 // This is thermistor 4
*  #define TEMP_SENSOR_BED 80 // This is not used thermistor

At this example we asigned to therm0 80 ( 3950 thermistor 100K - 4k7 Pull up and 10Uf Capacitor )(RAMPS STYLE).                                                                             
In basic example selected thermistor is PIN A0 and therm0 (thermistor therm1(A0,0)).                                                                    
therm 1 is an 3950 thermistor connected in A0 pin with 4k7 pullup and 10Uf capacitor to GND.                                                          



## Temperature sensors available

 *     0 : not used
 *     1 : 100k thermistor - best choice for EPCOS 100k (4.7k pullup)
 *     2 : 200k thermistor - ATC Semitec 204GT-2 (4.7k pullup)
 *     3 : Mendel-parts thermistor (4.7k pullup)
 *     4 : 10k thermistor !! do not use it for a hotend. It gives bad resolution at high temp. !!
 *     5 : 100K thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (4.7k pullup)
 *     6 : 100k EPCOS - Not as accurate as table 1 (created using a fluke thermocouple) (4.7k pullup)
 *     7 : 100k Honeywell thermistor 135-104LAG-J01 (4.7k pullup)
 *     71: 100k Honeywell thermistor 135-104LAF-J01 (4.7k pullup)
 *     8 : 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup)
 *     9 : 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)
 *     10 : 100k RS thermistor 198-961 (4.7k pullup)
 *     11 : 100k beta 3950 1% thermistor (4.7k pullup)
 *     12 : 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup) (calibrated for Makibox hot bed)
 *     13 : 100k Hisens 3950  1% up to 300°C for hotend "Simple ONE " & "Hotend "All In ONE"
 *     20 : the PT100 circuit found in the Ultimainboard V2.x
 *     60 : 100k Maker's Tool Works Kapton Bed Thermistor beta=3950
 *     66 : 4.7M High Temperature thermistor from Dyze Design
 *     70 : the 100K thermistor found in the bq Hephestos 2
 *     75 : 100k Generic Silicon Heat Pad with NTC 100K MGB18-104F39050L32 thermistor
 *     80 : 3950 thermistor 100K - 4k7 Pull up and 10Uf Capacitor
 
        1k ohm pullup tables - This is atypical, and requires changing out the 4.7k pullup for 1k.
                             (but gives greater accuracy and more stable PID)

 *     51 : 100k thermistor - EPCOS (1k pullup)
 *     52 : 200k thermistor - ATC Semitec 204GT-2 (1k pullup)
 *     55 : 100k thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (1k pullup)
 
         Use these for Testing or Development purposes. NEVER for production machine.

*     998 : Dummy Table that ALWAYS reads 25°C or the temperature defined below.
*     999 : Dummy Table that ALWAYS reads 100°C or the temperature defined below.

## Acknowledgements

*  [Marlin Firmware](http://marlinfw.org/)

## More information

*  [Marlin usage](http://marlinfw.org/docs/configuration/configuration.html) 

## Authors

* **Miguel A. Califa U.** - [*GitHub*](https://github.com/miguel5612) - [CV](https://scienti.colciencias.gov.co/cvlac/visualizador/generarCurriculoCv.do?cod_rh=0000050477)
* **Carlos M. Pallares C.** - [*GitHub*]() - [CV]() 




<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/miguel5612/ThermistorLibrary.svg?style=flat-square
[contributors-url]: https://github.com/miguel5612/ThermistorLibrary/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/miguel5612/ThermistorLibrary.svg?style=flat-square
[forks-url]: https://github.com/miguel5612/ThermistorLibrary/network/members
[stars-shield]: https://img.shields.io/github/stars/miguel5612/ThermistorLibrary.svg?style=flat-square
[stars-url]: https://github.com/miguel5612/ThermistorLibrary/stargazers
[issues-shield]: https://img.shields.io/github/issues/miguel5612/ThermistorLibrary.svg?style=flat-square
[issues-url]: https://github.com/miguel5612/ThermistorLibrary/issues
[license-shield]: https://img.shields.io/github/license/miguel5612/ThermistorLibrary.svg?style=flat-square
[license-url]: https://github.com/miguel5612/ThermistorLibrary/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=flat-square&logo=linkedin&colorB=555
[build-url]: https://travis-ci.org/dwyl/esta.svg?branch=master
[linkedin-url]: https://www.linkedin.com/in/miguel5612
[product-screenshot]: images/screenshot.png
