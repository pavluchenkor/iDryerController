# Firmware

A universal solution for filament drying. Can be built from inexpensive and readily available components, such as an old heater block heater, an unused fan, and a heat sink. A box from a hardware store and any insulation material can serve as the enclosure. The list of components is limited only by imagination.

Capabilities:
- dry filament at temperatures up to 110°C
- dry at a selected temperature for a specified duration
- regulate heater power and maintain the required air temperature in the dryer with high precision
- ventilate the chamber at specified intervals
- display air temperature, heating element temperature, and humidity
- transition to storage mode upon drying completion and maintain low heat indefinitely
- store at a selected temperature
- save electrical energy when the specified humidity is reached, enable/disable drying based on HUMIDITY_HYSTERESIS parameter
- includes presets for commonly used plastics
- automatically tune PID parameters
- weigh the spool and display remaining filament on screen
- generate a high signal on a pin for connection to the filament sensor input of a printer board

Hardware features:
- control of almost any heating element 24V/220V
- control of the chamber circulation fan via PWM
- version for 24V operation (also works at 12V) up to version 4.3
- version for 220V operation
- use of inexpensive Chinese weight modules to obtain reasonably accurate spool weight measurements

Board designs are hosted on [jlcpcb.com](https://oshwlab.com/svet_team/idryer)

Firmware features:
- the firmware file contains all possible variants; firmware configuration is edited in the configuration.h file
- firmware for the Arduino bootloader / not the safest option despite numerous checks and protections
  - support ended starting with v0.5.*; functionality is not tested; all references in the firmware are relics
- firmware for the MiniCore core
  - watch dog timer supported / will restart the firmware in case of software failure
  - Russian and English languages
  - PID control
  - drying and storage modes
  - support for selecting fan PWM frequencies
  - test for optimal fan frequencies
  - support for up to 4 weight modules on the HX711 ADC
  - weight polling at 10 and 80 times per second
  - display of spool weight on screen
  - tare weight input in menu
  - end-of-filament alerting via visual, audible, and filament sensor output
  - [servo drive support](https://t.me/iDryer/361) for regular chamber ventilation
  - custom preset names
  - extended menu and functions
  - firmware security

!!! warning annotate "IMPORTANT"

    :arrow_forward:</i> During operation, remember to follow the safety rules for household electrical appliances and heating devices.

    :arrow_forward:</i> The algorithms used cannot guarantee 100% safety of the device.


[Firmware](https://github.com/pavluchenkor/iDryerController)

[Telegram group :fontawesome-solid-paper-plane:](https://t.me/iDryer){ .md-button }

[Contributing :material-file-edit:](https://github.com/pavluchenkor/iDryerController)
