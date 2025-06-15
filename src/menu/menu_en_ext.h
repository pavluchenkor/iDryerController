#include <Arduino.h>
#include "version.h"

const char PROGMEM s0[]  = "FIRMWARE";
const char PROGMEM s1[]  = "IDRYER";
const char PROGMEM s2[]  = "VERSION";
// const char PROGMEM s3[]  = "0.1.0";
const char PROGMEM s4[]  = "COOLING";
const char PROGMEM s5[]  = "SAVED";
const char PROGMEM s6[]  = "AIR";
const char PROGMEM s7[]  = "HEATER";
const char PROGMEM s8[]  = "HUMIDITY";
const char PROGMEM s9[]  = "ERROR";
const char PROGMEM s10[]  = "CHECK";
const char PROGMEM s11[]  = "THERMISTOR";
const char PROGMEM s12[]  = "REBOOT";
const char PROGMEM s13[]  = "REMOVE";
const char PROGMEM s14[]  = "SPOOL";
const char PROGMEM s15[]  = "SETUP";
const char PROGMEM s16[]  = "PUT";
const char PROGMEM s17[]  = "WEIGHT";
const char PROGMEM s18[]  = "G";
const char PROGMEM s19[]  = "C";
const char PROGMEM s20[]  = "FLASH";
const char PROGMEM s21[]  = "PART 2";
const char PROGMEM s22[]  = "FREQUENCY";

const char i0[] PROGMEM = "MENU";
const char i1[] PROGMEM = "DRYING";
const char i2[] PROGMEM = "TEMPERATURE";
const char i3[] PROGMEM = "TIME";
const char i4[] PROGMEM = "START";
const char i5[] PROGMEM = "STORAGE";
// const char i6[] PROGMEM = "TEMPERATURE";
const char i7[] PROGMEM = "HUMIDITY";
// const char i8[] PROGMEM = "START";
const char i9[] PROGMEM = "PRESETS";
const char i10[] PROGMEM = PRESET_NAME_1;
// const char i11[] PROGMEM = "TEMPERATURE";
// const char i12[] PROGMEM = "TIME";
// const char i13[] PROGMEM = "START";
const char i14[] PROGMEM = PRESET_NAME_2;
// const char i15[] PROGMEM = "TEMPERATURE";
// const char i16[] PROGMEM = "TIME";
// const char i17[] PROGMEM = "START";
const char i18[] PROGMEM = PRESET_NAME_3;
// const char i19[] PROGMEM = "TEMPERATURE";
// const char i20[] PROGMEM = "TIME";
// const char i21[] PROGMEM = "START";
const char i22[] PROGMEM = "SETTINGS";
const char i23[] PROGMEM = "PID";
const char i24[] PROGMEM = "KP";
const char i25[] PROGMEM = "KI";
const char i26[] PROGMEM = "KD";
const char i27[] PROGMEM = "KF";
const char i28[] PROGMEM = "TIME/MS";
const char i29[] PROGMEM = "AUTOPID";
// const char i29_0[] PROGMEM = "TEMPERATURE";
// const char i29_1[] PROGMEM = "START";
const char i31[] PROGMEM = "AIRFLOW";
const char i32[] PROGMEM = "DELTA";
const char i33[] PROGMEM = "SERVO";
const char i34[] PROGMEM = "CLOSED";
const char i35[] PROGMEM = "OPEN";
const char i36[] PROGMEM = "CORNER";
const char i37[] PROGMEM = "TEST";
const char i38[] PROGMEM = "SAVE";
#if SCALES_MODULE_NUM > 0
const char i39[] PROGMEM = "SCALE";
const char i40[] PROGMEM = "SPOOL1";
const char i41[] PROGMEM = "TARE";
const char i42[] PROGMEM = "SET";
#endif
#if SCALES_MODULE_NUM > 1
const char i43[] PROGMEM = "SPOOL2";
// const char i44[] PROGMEM = "ТАРА";
// const char i45[] PROGMEM = "УСТАНОВКА";
#endif
#if SCALES_MODULE_NUM > 2
const char i46[] PROGMEM = "SPOOL3";
// const char i47[] PROGMEM = "ТАРА";
// const char i48[] PROGMEM = "УСТАНОВКА";
#endif
#if SCALES_MODULE_NUM > 3
const char i49[] PROGMEM = "SPOOL4";
// const char i50[] PROGMEM = "ТАРА";
// const char i51[] PROGMEM = "УСТАНОВКА";
#endif