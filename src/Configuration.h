#ifndef CONFIGURATION_H
#define CONFIGURATION_H

// При старте висит на заставке и пищит коротко(0,3с) - ошибка инициализации BME
// При старте висит на заставке и пищит продолжительно(1с) - ошибка температуры с термистора

/* !!!!!!!!!!!!!!!!!!!!!!!ERROR CODE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

01 void heaterOFF();
02 void heater(uint16_t Output, uint16_t &dimmer);
03 void heaterON();
04 void updateIDyerData();
05
06 void screen(struct subMenu *subMenu);
07 void controlsHandler(const menuS constMenu[], uint16_t editableMenu[], const ptrFunc functionMenu[], struct subMenu *subMenu);
08 void submenuHandler(const menuS constMenu[], uint8_t menuSize, struct subMenu *subMenu);
09 void piii(uint16_t time_ms);
10 void dryStart();
11 void storageStart();
12 void autoPidM();
13
14 uint32_t readError();
15 void setError(uint8_t errorCode);
16 void displayPrint(struct subMenu *subMenu);
17 void displayPrintMode();
18 // PID TUNING;
20 void scaleShow();
21
22 // CASE MENU
23 // CASE DRY
24 // CASE STORAGE
25 void autoPid();
26 // NTC MIN
27 // NTC MAX
28 // BME MIN
29 // BME MAX
30 // ADC ERROR
31 // iDryer.getData

!!!!!!!!!!!!!!!!!!!!!!!ERROR CODE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/

/******************************************************************************
PCB - https://oshwlab.com/svet_team/idryer
 ******************************************************************************/

/**********************
0 - русский
1 - english
 ********************/
#define LANG 1

/**********************
Sound notification when the heater is switched on
Comment if it bothers you that the
buzzer beeps every time the heater comes on
**********************/
// #define HEATER_ON_SOUND_NOTIFICATION

/*****************************************************************************************************
--NORMAL IS 4.7kohm PULLUP!-- 1kohm pullup can be used on hotend sensor, using correct resistor and table

   -3 : thermocouple with MAX31855 (only for sensor 0)
   -2 : thermocouple with MAX6675 (only for sensor 0)
   -1 : thermocouple with AD595
    0 : not used
    1 : 100k thermistor - best choice for EPCOS 100k (4.7k pullup)
    2 : 200k thermistor - ATC Semitec 204GT-2 (4.7k pullup)
    3 : Mendel-parts thermistor (4.7k pullup)
    4 : 10k thermistor !! do not use it for a hotend. It gives bad resolution at high temp. !!
    5 : 100K thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (4.7k pullup)
    6 : 100k EPCOS - Not as accurate as table 1 (created using a fluke thermocouple) (4.7k pullup)
    7 : 100k Honeywell thermistor 135-104LAG-J01 (4.7k pullup)
   71 : 100k Honeywell thermistor 135-104LAF-J01 (4.7k pullup)
    8 : 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup)
    9 : 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)
   10 : 100k RS thermistor 198-961 (4.7k pullup)
   11 : 100k beta 3950 1% thermistor (4.7k pullup)
   12 : 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup) (calibrated for Makibox hot bed)
   13 : 100k Hisens 3950  1% up to 300°C for hotend "Simple ONE " & "Hotend "All In ONE"
   20 : the PT100 circuit found in the Ultimainboard V2.x
   60 : 100k Maker's Tool Works Kapton Bed Thermistor beta=3950
   66 : 4.7M High Temperature thermistor from Dyze Design
   70 : the 100K thermistor found in the bq Hephestos 2
   75 : 100k Generic Silicon Heat Pad with NTC 100K MGB18-104F39050L32 thermistor
 *
      1k ohm pullup tables - This is atypical, and requires changing out the 4.7k pullup for 1k.
                             (but gives greater accuracy and more stable PID)
   51 : 100k thermistor - EPCOS (1k pullup)
   52 : 200k thermistor - ATC Semitec 204GT-2 (1k pullup)
   55 : 100k thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (1k pullup)
 *
 1047 : Pt1000 with 4k7 pullup
 1010 : Pt1000 with 1k pullup (non standard)
  147 : Pt100 with 4k7 pullup
  148 : E3D Pt100 with 4k7 pullup
  110 : Pt100 with 1k pullup (non standard)
 *
        Use these for Testing or Development purposes. NEVER for production machine.
  998 : Dummy Table that ALWAYS reads 25°C or the temperature defined below.
  999 : Dummy Table that ALWAYS reads 100°C or the temperature defined below.
 */

#define TEMP_SENSOR_0 5

#define TMP_MIN 1
#define TMP_MAX 130
#define HUMIDITY_HYSTERESIS 5
#define TEMP_HYSTERESIS 5

/**********************
110 Celsius enabler
100 or 110
 ********************/
 
 #define CE 110 //*


// Linear Temperature Calibration for BME280
#define MIN_CALIB_TEMP 28           // Regular ambient temperature
#define MAX_CALIB_TEMP CE           //!! Do not edit 
#define REAL_CALIB_TEMP_MIN 28      // BME280 readings at regular air temperature
#define REAL_CALIB_TEMP_MAX 121     // BME280 readings during thermocouple calibration

// Polynomial Temperature Calibration for BME280
// #define COEFF_A 0.0
// #define COEFF_B 0.0
// #define COEFF_C 0.0
// #define COEFF_D 0.0

// #define COEFF_A 7.8e-05
// #define COEFF_B -0.010865
// #define COEFF_C 0.484942
// #define COEFF_D -3.841228


/**********************
Select the power-on algorithm.
In storage mode, switching on can only be done
according to humidity hysteresis or switching on will be
affected by humidity and temperature hysteresis
0 only humidity
1 humidity and temperature
 **********************/

#define ACTIVATION_HYSTERESIS_MODE 1

/**********************
Autopid attemption
aprox 1min per attempt
 ********************/
#define AUTOPID_ATTEMPT 10

/**********************
Manual PID setup
if OVERWRITE_PID - 0, pid, after burning, will be default
 **********************/
#define OVERWRITE_PID 0
#define K_PROPRTIONAL 20  //
#define K_INTEGRAL 1      //
#define K_DERIVATIVE 40   //
#define K_SAMPLE_TIME 300 //
/*********************
Pid type
P_ON_M specifies that Proportional on Measurement be used
P_ON_E (Proportional on Error) is the default behavior
 ********************/
#define PID_TYPE P_ON_E
/*********************
Your revision PCB
1 - 220v revision
2 - 24v revision
 ********************/
#define REV 1

/**********************
1 - SH1106
2 - SSD1306
**********************/
#define SCREEN 2

/**********************
Screen rotation
0 - norm,
1 - rev
**/
#define SCREEN_FLIP 0

/**********************
Encoder Type
EB_STEP4_LOW
EB_STEP4_HIGH
EB_STEP2
EB_STEP1
********************/
#define MY_ENCODER_TYPE EB_STEP4_LOW // 0
#define ENCODER_FAST_INCREMENT 10    // 10

/**********************
Encoder direction
0 - normal
1 - reverse
 ********************/
#define ENCODER_REVERSE 0

/**********************
screen update time, change if you know what you're doing.
 ********************/
#define SCREEN_UPADATE_TIME 5000

/**********************
FAN PWM frequency
62500
31400
8000
4000
2000
980
245
122
60
30
********************/
#define PWM_11_FREQUENCY 122
/*************************
Temporarily uncomment to perform a
test at the specified PWM frequency
 *************************/
// #define PWM_TEST

/**********************
SERVO
the minimum and maximum pulse widths that will suit most servos (in us)
https://en.wikipedia.org/wiki/Servo_control
SERVO_CUCKOO - Servo sound notification time 0 - off, 50-100 - normal
 ********************/
#define SERVO_MIN_PULSE 300
#define SERVO_MAX_PULSE 2000
#define SERVO_PERIOD_MS 20
#define SERVO_CUCKOO 50 // 0

/**********************
if you change here, do an upload isp and upload eep anyway
if your SCALES_MODULE_NUM not 0 do this:
   1 pio run -e EEP -t fullclean -t uploadeep
   2 set #define AUTOPID_RUN 1
   2.1 CTRL + S
   3 pio run -e EEP -t fullclean -t upload
   4 wait for autopid to finish setting up
   5 set #define AUTOPID_RUN 0
   5.1 CTRL + S
   6 pio run -e EEP -t fullclean -t upload
 ********************/
#define AUTOPID_RUN 0
#define SCALES_MODULE_NUM 2

#define FILAMENT_SENSOR_ON                   //
#define ALERT_MASS 100                       // Beep by buzzer when less than this amount of filament left
#define FILAMENT_SENSOR_MASS 30              // [20 - 100] Send filament sensor signal (HIGH level) when less than this amount of filament left
#define FILAMENT_REFERENCE_FLOW_RATE_MASS 15 // Reference weight of the filament consumption at which the tracking of the spool weight change starts

#define ALERT_MASS_PIII_TIME 5            // sec
#define FILAMENT_SENSOR_MASS_PIII_TIME 15 // sec

#define MENU_SCALE_SWITCH_TIME 5000

/**********************
My presets name
No more than three capital letter
Choose the letters that
correspond to the set language
Letters written in another
language, will not be displayed
 *********************/

#if LANG == 0
#define PRESET_NAME_1 "ПЛА"
#define PRESET_NAME_2 "ПЭТГ"
#define PRESET_NAME_3 "РА6"
#elif LANG == 1
#define PRESET_NAME_1 "PLA"
#define PRESET_NAME_2 "PETG"
#define PRESET_NAME_3 "PA6"
#endif

#endif
