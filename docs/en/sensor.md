## Air Temperature Sensor Calibration

The BME280 air temperature sensors used in the system exhibit non-linearity at the edges of their measurement range. To compensate for this, a linear correction algorithm is applied.

This algorithm gets its parameters from the `configuration.h` section:

```
#define MIN_CALIB_TEMP 70
#define MAX_CALIB_TEMP CE
#define REAL_CALIB_TEMP_MIN 70
#define REAL_CALIB_TEMP_MAX 110
```

Where:

* `MIN_CALIB_TEMP` - temperature below which readings are not corrected
* `MAX_CALIB_TEMP` - upper limit of the range where calibration is applied
* `REAL_CALIB_TEMP_MIN` - actual temperature from a reference sensor at `MIN_CALIB_TEMP`
* `REAL_CALIB_TEMP_MAX` - actual temperature from a reference sensor at `MAX_CALIB_TEMP`

### Test Preparation

Set up the system with a connected `bmeTemp` sensor and a reference thermistor.

Ensure calibration parameters are defined:

```
MIN_CALIB_TEMP = 70
REAL_CALIB_TEMP_MIN = 70
MAX_CALIB_TEMP = 100 or 110 (depending on configuration)
REAL_CALIB_TEMP_MAX = 110 (depending on configuration)
```

Ensure precise temperature input can be set.

### Test Steps

**Checking the non-calibrated zone:**

* Set temperature to 65°C and check with the reference sensor
* Repeat for 70°C, 75°C, etc.
* Identify the temperature at which the reference and internal sensors begin to diverge
* Record this temperature into `MIN_CALIB_TEMP` and `REAL_CALIB_TEMP_MIN`

**Checking the calibration start point:**

* Set the maximum calibration temperature to `CE`
* Wait until target temperature is reached
* Record the reading from the reference sensor into `REAL_CALIB_TEMP_MAX`
* Save the changes
* Flash the controller using the command:

```
pio run -t clean -t upload
```

---

## Scale Calibration

### Scale Installation

To ensure correct operation, observe the following:

* Proper installation
* Proper wiring

#### Installation

Make sure the weighing module is 1.5-2mm away from the mounting plate and that the potting compound does not touch the plate.

<!--
#### Wiring
Typically, the scale module uses a black wire (-) and red wire (+) connected to E- and E+ respectively. The remaining two wires connect to A+ and A-. To identify them:
1. Connect black and red wires to power
2. Set multimeter to DC voltage mode
3. Connect the multimeter's black lead to the black wire (GND)
4. Probe the white wire with the red lead
5. Press on the load cell. If the reading increases, it's A+; if it decreases, it's A-.
-->

#### Calibration

Remove any filament spool from the module being calibrated.

In the `SCALE` menu, select `SET`.

Wait for the message `PUT WEIGHT 1000G` and place a 1000g calibration weight on the spool holder.

Repeat for each module.

If the displayed weight is incorrect, check the following:

* Correct wiring
* Secure mounting
* Position of the zero resistor on the HX711 module
* Temperature of the ADC chip (should not be hot)
