#include <Arduino.h>
#include "version.h"

const char PROGMEM s0[] = "ПРОШИВКА";
const char PROGMEM s1[] = "АЙДРАЙ";
const char PROGMEM s2[] = "ВЕРСИЯ";
// const char PROGMEM s3[]  = "0.1.0";
const char PROGMEM s4[] = "ОХЛАЖДЕНИЕ";
const char PROGMEM s5[] = "СОХРАНЕНО";
const char PROGMEM s6[] = "ВОЗДУХ";
const char PROGMEM s7[] = "НАГРЕВ";
const char PROGMEM s8[] = "ВЛАЖНОСТЬ";
const char PROGMEM s9[] = "ОШИБКА";
const char PROGMEM s10[] = "ПРОВЕРЬ";
const char PROGMEM s11[] = "ТЕРМИСТОР";
const char PROGMEM s12[] = "ПЕРЕГРУЗИ";
const char PROGMEM s13[] = "УДАЛИ";
const char PROGMEM s14[] = "КАТУШКА";
const char PROGMEM s15[] = "УСТАНОВКА";
const char PROGMEM s16[] = "УСТАНОВИ";
const char PROGMEM s17[] = "ВЕС";
const char PROGMEM s18[] = "Г";
const char PROGMEM s19[] = "С";
const char PROGMEM s20[]  = "ПРОШЕЙ";
const char PROGMEM s21[]  = "ЧАСТЬ 2";
const char PROGMEM s22[]  = "ЧАСТОТА";

const char i0[] PROGMEM = "МЕНЮ";
const char i1[] PROGMEM = "СУШКА";
const char i2[] PROGMEM = "ТЕМПЕРАТУРА";
const char i3[] PROGMEM = "ВРЕМЯ";
const char i4[] PROGMEM = "СТАРТ";
const char i5[] PROGMEM = "ХРАНЕНИЕ";
// ; const char i6[] PROGMEM = "ТЕМПЕРАТУРА";
const char i7[] PROGMEM = "ВЛАЖНОСТЬ";
// const char i8[] PROGMEM = "СТАРТ";
const char i9[] PROGMEM = "ПРЕСЕТЫ";
const char i10[] PROGMEM = PRESET_NAME_1;
// ; const char i11[] PROGMEM = "ТЕМПЕРАТУРА";
// ; const char i12[] PROGMEM = "ВРЕМЯ";
// ; const char i13[] PROGMEM = "СТАРТ";
const char i14[] PROGMEM = PRESET_NAME_2;
// ; const char i15[] PROGMEM = "ТЕМПЕРАТУРА";
// ; const char i16[] PROGMEM = "ВРЕМЯ";
// ; const char i17[] PROGMEM = "СТАРТ";
const char i18[] PROGMEM = PRESET_NAME_3;
// ; const char i19[] PROGMEM = "ТЕМПЕРАТУРА";
// ; const char i20[] PROGMEM = "ВРЕМЯ";
// ; const char i21[] PROGMEM = "СТАРТ";
const char i22[] PROGMEM = "НАСТРОЙКИ";
const char i23[] PROGMEM = "ПИД";
const char i24[] PROGMEM = "КП";
const char i25[] PROGMEM = "КИ";
const char i26[] PROGMEM = "КД";
const char i27[] PROGMEM = "КФ";
const char i28[] PROGMEM = "ВРЕМЯ/МС";
const char i29[] PROGMEM = "АВТОПИД";
// ; const char i29_0[] PROGMEM = "ТЕМПЕРАТУРА";
// ; const char i29_1[] PROGMEM = "СТАРТ";
const char i31[] PROGMEM = "ОБДУВ";
const char i32[] PROGMEM = "ДЕЛЬТА";
const char i33[] PROGMEM = "СЕРВО";
const char i34[] PROGMEM = "ЗАКРЫТО";
const char i35[] PROGMEM = "ОТКРЫТО";
const char i36[] PROGMEM = "УГОЛ";
const char i37[] PROGMEM = "ТЕСТ";
const char i38[] PROGMEM = "СОХРАНИТЬ";
#if SCALES_MODULE_NUM > 0
const char i39[] PROGMEM = "ВЕСЫ";
const char i40[] PROGMEM = "КАТУШКА1";
const char i41[] PROGMEM = "ТАРА";
const char i42[] PROGMEM = "УСТАНОВКА";
#endif
#if SCALES_MODULE_NUM > 1
const char i43[] PROGMEM = "КАТУШКА2";
// const char i44[] PROGMEM = "ТАРА";
// const char i45[] PROGMEM = "УСТАНОВКА";
#endif
#if SCALES_MODULE_NUM > 2
const char i46[] PROGMEM = "КАТУШКА3";
// const char i47[] PROGMEM = "ТАРА";
// const char i48[] PROGMEM = "УСТАНОВКА";
#endif
#if SCALES_MODULE_NUM > 3
const char i49[] PROGMEM = "КАТУШКА4";
// const char i50[] PROGMEM = "ТАРА";
// const char i51[] PROGMEM = "УСТАНОВКА";
#endif