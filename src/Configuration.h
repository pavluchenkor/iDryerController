#ifndef CONFIGURATION_H
#define CONFIGURATION_H

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
#define TEMP_SENSOR_0 5 // This is thermistor 0

/** Screen rotation 
 * 0 - norm, 
 * 1 - rev 
 * **/
#define SCREEN_FLIP 1

/** 0-255 PWM 8bit
 * If your heater is very powerful,
 * for example more than 300-400 watts,
 * this can be reduced to 200-150
 * **/
#define HEATER_MAX 255

/** Autopid attemption **/
#define AUTOPID_ATTEMPT 20

/**
 * Your revision PCB
 * 1 - 220v revision
 * 2 - 24v revision
 **/
#define REV 1

/**
 * 1 - SH1106
 * 2 - SSD1306
 */
#define SCREEN 2

#endif
