## Wiring

Connect the Arduino Nano to the USBASP programmer.
(ArduinoISP has not been tested with the existing `platformio.ini` sections and may require manual configuration.)

![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/pins.png)

![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/usbasp10.jpg)

> Note: Pinout may differ on some connectors.

![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/img001.png)

## Compilation and Flashing

!!! info annotate "configuration.h"
\:arrow\_forward: Configure your firmware in the `configuration.h` file.

!!! warning annotate "Compilation errors"
If you encounter memory size mismatch errors during compilation, run:
`     pio update
    pio pkg update
    `

### Select Number of Scale Modules

=== "2 modules"

    ```
    #define SCALES_MODULE_NUM 2
    ```

=== "3 modules"

    ```
    #define SCALES_MODULE_NUM 3
    ```

=== "4 modules"

    ```
    #define SCALES_MODULE_NUM 4
    ```

### 1. Flashing MiniCore Bootloader

You’ll need a USBASP programmer.

#### 2.1 Fuse Configuration

=== "Atmega328P"

    ```
    pio run -e fuses -t fuses
    ```

=== "Atmega328PB"

    ```
    pio run -e fuses -t fuses
    ```

#### 2.2 Flash EEPROM

=== "Atmega328P"

    ```
    pio run -e EEP -t uploadeep
    ```

=== "Atmega328PB"

    ```
    pio run -e EEPPB -t uploadeep
    ```

### Flashing the Microcontroller

=== "Atmega328P"

    ```
    pio run -e EEP -t upload
    ```

=== "Atmega328PB"

    ```
    pio run -e EEPPB -t upload
    ```

### Autopid Calibration (Optional)

Performed in two steps:

Edit `configuration.h`:

```
#define AUTOPID_RUN 1
#define SCALES_MODULE_NUM X (X = number of scale modules)
```

Then flash the firmware (step 2.3). PID calibration will begin automatically. Once completed, the screen will display "Flash Part 2". Now set:

```
#define AUTOPID_RUN 0
```

Then reflash (step 2.3).

### Fan Test Firmware (Optional)

In `configuration.h`, uncomment:

```
// #define PWM_TEST
```

Then flash (step 2.3).

The fan will cycle through all supported PWM frequencies with 100-10% duty cycles. Once complete, choose the desired frequency:

```
#define PWM_11_FREQUENCY
```

Then comment out:

```
#define PWM_TEST
```

And reflash (step 2.3).

!!! warning annotate "Startup Errors"
If errors occur during startup or runtime, refer to `configuration.h`.

## Tutorial Video

<div class="video-wrapper">
  <iframe width="1280" height="720" src="https://www.youtube.com/embed/psw30jXlxPQ" frameborder="0" allowfullscreen></iframe>
</div>

### Related Links

* [VS Code](https://code.visualstudio.com/Download)
* [USB Driver (Zadig)](https://zadig.akeo.ie/)
* [Firmware Source](https://github.com/pavluchenkor/iDryerController)

[Join Telegram Group \:fontawesome-solid-paper-plane:](https://t.me/iDryer){ .md-button }

[Contribute \:material-file-edit:](https://github.com/pavluchenkor/iDryerController)
