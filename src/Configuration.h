#ifndef CONFIGURATION_H
#define CONFIGURATION_H

/* !!!!!!!!!!!!!!!!!!!!!!!ERROR CODE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
// /* 01 */ void heaterOFF();
// /* 02 */ void heaterON(uint16_t Output, uint16_t &dimmer);
// /* 03 */ void servoPulse(int pin, int angle);
// /* 04 */ void updateIDyerData();
// /* 05 */ void encoderSate(struct control *control);
// /* 06 */ void screen(struct subMenu *subMenu);
// /* 07 */ void controlsHandler(const menuS constMenu[], uint16_t editableMenu[], const ptrFunc functionMenu[], struct control *encoder, struct subMenu *subMenu);
// /* 08 */ void submenuHandler(const menuS constMenu[], uint8_t menuSize, struct subMenu *subMenu);
// /* 09 */ void piii(uint16_t time_ms);
// /* 10 */ void dryStart();
// /* 11 */ void storageStart();
// /* 12 */ void autoPidM();
// /* 13 */ uint32_t printError(uint32_t error);
// /* 14 */ uint32_t readError();
// /* 15 */ bool setError(uint8_t errorCode);
// /* 16 */ void displayPrint(struct subMenu *subMenu);
// /* 17 */ void displayPrintMode();
// /* 18 */ // PID TUNING;
// /* 20 */ // CASE OFF
// /* 21 */ // CASE ON
// /* 22 */ // CASE MENU
// /* 23 */ // CASE DRY
// /* 24 */ // CASE STORAGE
// /* 25 */ // CASE AUTOPID
// /* 26 */ // NTC MIN
// /* 27 */ // NTC MAX
// /* 28 */ // BME MIN
// /* 29 */ // BME MAX
// /* 30 */ // ADC ERROR
// /* 0 */  // ADC ACCUMULATED ERROR
// /* 31 */ // iDryer.getData
/* !!!!!!!!!!!!!!!!!!!!!!!ERROR CODE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/

/******************************************************************************
 * Если используется версия без замены бутлоатера ардуино то:
 * - нет ватчдога(это такая фигня которая в случае зависания ардуины перезагрузит ее и, как вариант, нагреватель не спалит дом)
 * - нет поддержки серв и как следствие...
 * - нет функции проветривания
 * - нет весов
 *
 * Перед прошивкой микроконтроллера сконфигурируй прошивку, все дефайны подписаны,
 * а если не подписаны, то лучше оставь как есть
 *
 * Для прошивки кастрата запускаем в терминале платформио команду
 * pio run -e nanoatmega328new -t upload
 * все, ардуина готова к использованию с платой https://oshwlab.com/svet_team/idryer
 *
 * Для прошивки ардуины в нормальный контроллер потребуется программатор USBasp,
 * можно воспльзоваться и usbisp или ардуиной прешитой в usbisp но это не тестировалось
 * подключаем по схеме //TODO
 * раскомментируем строку  #define WITH_BLACKJACK_AND_HOOKERS
 * В терминале платформио запускаем:
 * 1) pio run -e fuses_bootloader -t bootloader
 *    ардуина станет нормальным контроллером
 * 2) pio run -e EEP -t upload
 * все, ардуина готова к использованию с платой https://oshwlab.com/svet_team/idryer
 ******************************************************************************/

/**********************
 * if you have USBasp programmer,
 * make several easy steps
 * end uncomment WITH_BLACKJACK_AND_HOOKERS
 ********************/
#define WITH_BLACKJACK_AND_HOOKERS

/**********************
 * 0 - русский
 * 1 - english
 ********************/
#define LANG 0

/* eeprom */

/**
 * --NORMAL IS 4.7kohm PULLUP!-- 1kohm pullup can be used on hotend sensor, using correct resistor and table
 *
 *    -3 : thermocouple with MAX31855 (only for sensor 0)
 *    -2 : thermocouple with MAX6675 (only for sensor 0)
 *    -1 : thermocouple with AD595
 *     0 : not used
 *     1 : 100k thermistor - best choice for EPCOS 100k (4.7k pullup)
 *     2 : 200k thermistor - ATC Semitec 204GT-2 (4.7k pullup)
 *     3 : Mendel-parts thermistor (4.7k pullup)
 *     4 : 10k thermistor !! do not use it for a hotend. It gives bad resolution at high temp. !!
 *     5 : 100K thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (4.7k pullup)
 *     6 : 100k EPCOS - Not as accurate as table 1 (created using a fluke thermocouple) (4.7k pullup)
 *     7 : 100k Honeywell thermistor 135-104LAG-J01 (4.7k pullup)
 *    71 : 100k Honeywell thermistor 135-104LAF-J01 (4.7k pullup)
 *     8 : 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup)
 *     9 : 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)
 *    10 : 100k RS thermistor 198-961 (4.7k pullup)
 *    11 : 100k beta 3950 1% thermistor (4.7k pullup)
 *    12 : 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup) (calibrated for Makibox hot bed)
 *    13 : 100k Hisens 3950  1% up to 300°C for hotend "Simple ONE " & "Hotend "All In ONE"
 *    20 : the PT100 circuit found in the Ultimainboard V2.x
 *    60 : 100k Maker's Tool Works Kapton Bed Thermistor beta=3950
 *    66 : 4.7M High Temperature thermistor from Dyze Design
 *    70 : the 100K thermistor found in the bq Hephestos 2
 *    75 : 100k Generic Silicon Heat Pad with NTC 100K MGB18-104F39050L32 thermistor
 *
 *       1k ohm pullup tables - This is atypical, and requires changing out the 4.7k pullup for 1k.
 *                              (but gives greater accuracy and more stable PID)
 *    51 : 100k thermistor - EPCOS (1k pullup)
 *    52 : 200k thermistor - ATC Semitec 204GT-2 (1k pullup)
 *    55 : 100k thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (1k pullup)
 *
 *  1047 : Pt1000 with 4k7 pullup
 *  1010 : Pt1000 with 1k pullup (non standard)
 *   147 : Pt100 with 4k7 pullup
 *   148 : E3D Pt100 with 4k7 pullup
 *   110 : Pt100 with 1k pullup (non standard)
 *
 *         Use these for Testing or Development purposes. NEVER for production machine.
 *   998 : Dummy Table that ALWAYS reads 25°C or the temperature defined below.
 *   999 : Dummy Table that ALWAYS reads 100°C or the temperature defined below.
 */
#define TEMP_SENSOR_0 6 // This is thermistor 0  /реборн сток EPCOS 100K B57560G104F - 6

#define TMP_MIN 1
#define TMP_MAX 120

/**********************
 * Autopid attemption
 * aprox 1min per attempt
 ********************/
#define AUTOPID_ATTEMPT 20

/**********************
 * Manual PID setup
 * if OVERWRITE_PID - 0, pid, after burning, will be default
 **********************/
#define OVERWRITE_PID 0
#define K_PROPRTIONAL 20  // 19287
#define K_INTEGRAL 1      // 1972
#define K_DERIVATIVE 40   //
#define K_SAMPLE_TIME 300 // Чем более инертная система тем больше времени примерно от 250(не менее 200) для легких и быстрых нагреватеелей до 1000 для толстых с длительным временем нагрева

/*********************
 * Your revision PCB
 * 1 - 220v revision
 * 2 - 24v revision
 ********************/
#define REV 1

/**********************
 * 1 - SH1106
 * 2 - SSD1306
 */
#define SCREEN 2

/**********************
 * Screen rotation
 * 0 - norm,
 * 1 - rev
 * **/
#define SCREEN_FLIP 1

/**********************
 * Encoder Type
 * EB_STEP4_LOW 0
 * EB_STEP4_HIGH 1
 * EB_STEP2 2
 * EB_STEP1 3
 * если энкодер невменяшка, попробуй поменять значение
 ********************/
#define MY_ENCODER_TYPE 2 // 0

/**********************
 * Encoder direction
 * 1
 * 0
 ********************/
#define ENCODER_REVERSE 1

/**********************
 * screen update time, change if you know what you're doing.
 ********************/
#define SCREEN_UPADATE_TIME 5000

/**********************
 * BUZZER
 * 180 pwm busser
 * 250 active buzzer
 ********************/
#define BUZZER_PWM 180

/**********************
 * FAN
 * 11 FAN connector on PCB 488Hz
 * 6  EXT connector on PCB 976Hz Not tested!
 ********************/
#define FAN_PIN 11

// /**********************
//  * FAN PWM frequency
//  * only on 11 FAN
//  * 62500
//  * 31400
//  * 8000
//  * 4000
//  * 2000
//  * 980
//  * 245
//  * 122
//  * 60
//  * 30
//  ********************/
#define PWM_11_FREQUENCY 122
/*************************
 * Temporarily uncomment to perform a
 * test at the specified PWM frequency
 *************************/
// #define PWM_TEST

/**********************
 * SERVO
 * the minimum and maximum pulse widths that will suit most servos (in us)
 * https://en.wikipedia.org/wiki/Servo_control
 * SERVO_CUCKOO - Servo sound notification time 0 - off, 50-100 - normal
 ********************/
#define SERVO_MIN_PULSE 600
#define SERVO_MAX_PULSE 1800
#define SERVO_PERIOD_MS 20
#define SERVO_CUCKOO 50 // 0

/**********************
 * 0
 * 1
 * 2
 * 3
 * 4
 *
 * if you change here, do an upload isp and upload eep anyway
 * if your SCALES_MODULE_NUM not 0 do this:
 *    1 pio run -e EEP -t fullclean -t uploadeep
 *    2 set #define AUTOPID_RUN 1
 *    3 pio run -e EEP -t fullclean -t upload
 *    4 wait for autopid to finish setting up
 *    5 set #define AUTOPID_RUN 0
 *    6 pio run -e EEP -t fullclean -t upload
 ********************/
#define AUTOPID_RUN 0
#define SCALES_MODULE_NUM 4
#define ALERT_MASS 100
#define FILAMENT_SENSOR_MASS 20
#define MENU_SCALE_SWITCH_TIME 7000

/**********************
 * My presets name
 * No more than three capital letter
 * Choose the letters that
 * correspond to the set language
 * Letters written in another
 * language, will not be displayed
 *********************/

#define PRESET_NAME_1 "ПЛА"
#define PRESET_NAME_2 "ПЕТГ"
#define PRESET_NAME_3 "РА6"

#endif
