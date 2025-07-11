## 0.5.8

Support for SHT31
SHT31:

* Accuracy: ±2% RH and ±0.3°C - better than most I2C sensors
* Response time: <8 seconds for humidity
* Operating range: up to 125°C and 0-100% RH
* Resistant to condensation - crucial when drying down to 0%
* Calibrated and stable - no manual calibration required
  Unlike DHT22, AM2302, and similar sensors, it performs reliably at high temperatures and doesn’t fail due to overheating.

## 0.5.7

Refactoring, improved operational stability

## 0.5.6

* 110°C option added in config
* Updated temperature control algorithm

## 0.5.5

(no changelog provided)

## 0.5.4

(no changelog provided)

## 0.5.3

* Added audio alerts for temperature sensor issues at startup:

  * Short beeps: BME startup issue (long cable or poor contact)
  * Long beeps: out-of-range thermistor values (wrong thermistor type, incorrect divider resistor, bad contact, or extreme temperature)
* Changed error accumulation logic for temperature and humidity readings
* Adjusted polling rate and data filtering for BME280
* Minor bug fixes

## 0.5.2

* Screen updates every 5 seconds when temperature or humidity changes
* Fixed compilation error for 24V version
* Fixed hysteresis on cooling [https://github.com/pavluchenkor/iDryerController/pull/6/files](https://github.com/pavluchenkor/iDryerController/pull/6/files)
* Corrected PETG to ПЭТГ [https://github.com/pavluchenkor/iDryerController/pull/7/files#:\~:text=define%20PRESET\_NAME\_2%20%22-,%D0%9F%D0%AD%D0%A2%D0%93,-%22](https://github.com/pavluchenkor/iDryerController/pull/7/files#:~:text=define%20PRESET_NAME_2%20%22-,%D0%9F%D0%AD%D0%A2%D0%93,-%22)

## 0.5.1

* Scale support added
* Frequency selection for scale polling: 10/80Hz
* Improved weight reading stability
* English language support added
* Non-blocking servo now runs on a timer
* Minor improvements and enhanced firmware stability

## 0.5.0

* Scale config implemented
* Median filter for scale data
* Auto-PID only runs when AUTOPID\_RUN != 1 if scales are in use
* Preset names moved to config
* Zero calibration added
* Tare weight can be set in the menu
* Replaced float with integer for mass; scales now round down to nearest whole number (e.g., 999.99 = 999)
* Bootloader removed as it’s no longer necessary

## 0.4.4

* Increment/decrement by 10 when rotating encoder quickly
* Improved input interface responsiveness
* Encoder and button use interrupts

## 0.4.3

* Servo test works without saving
* PWM frequency test added based on "Fan" variable
* Fixed servo functionality
* Preset modes functioning properly

## 0.4.2

* Servo test added

## 0.4.1

* Servo2 not supported
* min/max temp checks (tmpTemp <= ADC\_MIN || tmpTemp >= ADC\_MAX) only active during drying and storage

## 0.4.0

* Buzzer must be active type
* PWM frequency selection for pin 11 configurable in configuration.h (for noisy fans)
* HX711 library tested and integrated

## Version 0.3.6

* Refactored servo handling block

## Version 0.3.5

* First stable version for public use

```bash
git commit --allow-empty -m "0.3.6 final"
```
