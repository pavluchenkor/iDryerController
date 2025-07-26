### Purpose of the Device and General Information

The device is designed for drying and storing filament for 3D printing. It is supplied either as a KIT for self-assembly or fully assembled. The assembled unit is pre-configured and does not require any special setup.

### Operating the Device

#### Powering On and Off

Use the switch on the front panel to turn the device on or off. Upon startup, the current firmware version will be displayed. The fan may turn on if the chamber temperature exceeds 40℃-this is normal. No special action is needed before turning off the dryer.

#### Menu Navigation

All control is done via the menu using a rotary encoder.

* Turn clockwise - increase value or go to the next menu item
* Turn counterclockwise - decrease value or go to the previous menu item
* Short press - activate the selected menu item
* Long press - return to the previous menu or stop the current operation (drying or storage)

### Filament Drying Mode

Load the required number of filament spools into the dryer. Optionally feed the filament into PTFE tubes if you plan to print directly from the dryer. In the DRYING menu, set the temperature (`TEMPERATURE`) and duration in minutes (`TIME`). Start the drying mode by selecting `START`. To stop the drying, long-press the encoder.

During drying, the display alternates between weight readings and drying parameters:

* The top line shows the target temperature and remaining time
* `AIR` - air temperature inside the chamber
* `HEATER` - heating element temperature
* `HUMIDITY` - relative humidity inside the chamber

Drying mode operation algorithm:

1. Heat up to target temperature with fan at 100% speed
2. Maintain temperature with reduced fan speed for the set time
3. Switch to storage mode according to settings

### Filament Storage Mode

Storage mode is started manually via the STORAGE menu or automatically after drying. The device maintains target relative humidity, allowing you to start printing at any time without pre-drying. In the `STORAGE` menu, set the target relative humidity `HUMIDITY` and max allowable temperature `TEMPERATURE`. Start the mode by selecting `START`. Long-press the encoder to stop.

During storage, the display alternates between weight readings and storage parameters:

* The top line shows the target temperature and remaining time
* `AIR` - air temperature in the chamber
* `HEATER` - heater temperature
* `HUMIDITY` - relative humidity

### Adjusting Drying Parameters

Settings are in the `SETTINGS` menu. You can adjust airflow, temperature delta, damper, and PID. Save changes via `SETTINGS -> SAVE`.

#### Fan Settings

`AIRFLOW` - sets the fan speed (%) after reaching the working temperature to reduce noise. Recommended: at least 70%.

#### Temperature Difference Setting

`DELTA` - sets the target delta between heater and air temperature. Recommended: 20℃. A large delta may cause uneven heating; a small delta increases heating time.

#### Damper Servo Settings

`SETTINGS -> SERVO` - the dryer includes a servo-controlled damper for ventilation.

* `CLOSED` - minutes damper remains closed
* `OPEN` - minutes damper remains open
* `CORNER` - angle to open the damper (recommended: 85°)
* `TEST` - open/close damper for testing

#### PID Controller Settings

`PID` - for advanced users, normally does not require adjustment

* `kP` - proportional gain
* `kI` - integral gain
* `kD` - derivative gain
* `AUTOPID` - auto-calibration of PID

### Scale Setup

#### Scale Calibration

Calibrate each spool position separately. Use a stable 1 kg weight.

Go to `SCALE -> SPOOLX -> SET`. Remove all items when prompted. Then place the 1 kg weight when prompted.

#### Scale Taring

If you use the same spool type, you can set a tare weight to display net filament weight. This does not replace calibration.

Place an empty spool on the scale, wait for the weight to appear, and set it via `SCALE -> SPOOLX -> TARE`

### Troubleshooting

#### Error Message or Beeping on Startup

Short-press the encoder to exit to main menu. If it continues, power-cycle the device. If repeated, note the error code and report it.

#### Dryer Exits Drying Mode Unexpectedly

This can be due to low fan speed causing power supply issues. Set `SETTINGS -> AIRFLOW` to 75% or higher. If needed, set it to 100%.

#### Fan Makes Unusual Noise

May be caused by low airflow setting. Set `AIRFLOW` to 75% or more.

#### Inaccurate Scale Readings

Check the platform mount screws. Tighten if loose. Recalibrate the scales.

#### Settings Not Applied

After any change in `SETTINGS`, save with `SETTINGS -> SAVE`

#### Display Stays On After Power Off

This is normal behavior.

### Appendix 1: Recommended Drying Parameters

|               | Drying                      |         | Storage |          |
| :------------ | --------------------------- | ------- | ------- | -------- |
|               | Temp, ℃                     | Time, h | Temp, ℃ | Humidity |
| PLA           | 55                          | 8       | 50      | 15       |
| PETG          | 65                          | 8       | 60      | 15       |
| TPU           | 60-70                       | 8       | 60      | 15       |
| ABS/ASA       | 85                          | 8       | 70      | 15       |
| PA            | 80-110                      | 12      | 65      | 10       |
| PA-CF, Ultran | 90-110 (2-4h), then 80 (4h) | 6       | 70      | 10       |

!!! warning "Disclaimer"
These are general recommendations. Always check your filament manufacturer's specifications and assess drying effectiveness based on print quality.

```
Some manufacturers may understate recommended temperatures to simplify usage rather than ensure optimal results.
```

### Appendix 2: Menu Structure

* `DRYING`

  * `TEMPERATURE`
  * `TIME`
  * `START`
* `STORAGE`

  * `TEMPERATURE`
  * `HUMIDITY`
  * `START`
* `PRESETS`

  * `PLA`
  * `PETG`
  * `PA6`
* `SETTINGS`

  * `PID`

    * `kP`
    * `kI`
    * `kD`
    * `AUTOPID`
  * `AIRFLOW`
  * `DELTA`
  * `SERVO`

    * `CLOSED`
    * `OPEN`
    * `CORNER`
    * `TEST`
  * `SAVE`
* `SCALE`

  * `SPOOL1`

    * `TARE`
    * `SET`
  * `SPOOL2`

    * `TARE`
    * `SET`
