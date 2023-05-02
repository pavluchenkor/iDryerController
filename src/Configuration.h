#ifndef CONFIGURATION_H
#define CONFIGURATION_H

/******************************************************************************
 * Если используется версия без замены бутлоатера ардуино то:
 * - нет ватчдога(это такая фигня которая в случае зависания ардуины перезагрузит ее и, как вариант, нагреватель не рподпалит дом)
 * - нет поддержки серв и как следсттвие...
 * - нет функции провертривания
 * - нет весов
 * 
 * Перд прошивкой микроконтроллера сконфигурируй прошивку, все дефайны подписаны, 
 * а если не подписаны, то луше оставь как есть
 * 
 * Для прошивки кастрата запускаем в терминале плавтформио команду
 * pio run -e nanoatmega328new -t upload
 * все, ардуина готова к использованию с платой https://oshwlab.com/svet_team/idryer
 * 
 * Для прошивки ардуины в нормальный контроллер потребуется программатор USBasp,
 * можно восопльзоваться и usbisp или ардуиной прешитой в usbisp но это не тестировалось
 * подключаем по схеме //TODO
 * раскомментируем строку  #define WITH_BLACKJACK_AND_HOOKERS
 * В терминале платформио запускаем:
 * 1) pio run -e fuses_bootloader -t bootloader
 *    ардуина станет нормальным контроллером
 * 2) pio run -e eep -t upload
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
* 1 - not russian
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
#define TMP_MAX 100

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
#define K_PROPRTIONAL 10 // 49422
#define K_INTEGRAL 39 //3953
#define K_DERIVATIVE 23373
#define K_SAMPLE_TIME 500 // Чем более инертная система тем больше времени примерно от 250(не менее 200) для легких и быстрых нагреватеелей до 1000 для толстых с длительным временем нагрева

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
 *  0
 *  1
 * если энкодер невменяшка, попробуй поменять значение
 ********************/
#define MY_ENCODER_TYPE 1 //0

/**********************
 * Encoder direction 
 ********************/
// #define ENCODER_REVERSE

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
 * SERVO
 * the minimum and maximum pulse widths that will suit most servos (in us)
 * https://en.wikipedia.org/wiki/Servo_control
 * SERVO_CUCKOO - Servo sound notification time 0 - off, 50-100 - normal
 ********************/
#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2000
#define SERVO_PERIOD_MS 20
#define SERVO_CUCKOO 50 //0
#endif
