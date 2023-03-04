#include <Arduino.h>

#define DEF_MENU 0
#define DEF_DRY 1
#define DEF_TEMPERATURE 2
#define DEF_TIME 3
#define DEF_START 4
#define DEF_STORAGE 5
#define DEF_TEMPERATURE 6
#define DEF_HUMIDITY 7
#define DEF_START 8
#define DEF_PRESETS 9
#define DEF_PLA 10
#define DEF_TEMPERATURE 11
#define DEF_TIME 12
#define DEF_START 13
#define DEF_PETG 14
#define DEF_TEMPERATURE 15
#define DEF_TIME 16
#define DEF_START 17
#define DEF_ABS 18
#define DEF_TEMPERATURE 19
#define DEF_TIME 20
#define DEF_START 21
#define DEF_SETTINGS 22
#define DEF_PID 23
#define DEF_KP 24
#define DEF_KI 25
#define DEF_KD 26
#define DEF_AUTOPID 27
#define DEF_TIME_MS 28
#define DEF_TEMPERATURE 29
#define DEF_START 30
#define DEF_FAN 31
#define DEF_DELTA 32
#define DEF_SERVO 33
#define DEF_SERVO1 34
#define DEF_CLOSED 35
#define DEF_OPEN 36
#define DEF_CORNER 37
#define DEF_SERVO2 38
#define DEF_CLOSED 39
#define DEF_OPEN 40
#define DEF_CORNER 41
#define DEF_SAVE 42

struct menuS
{
    uint8_t id;
    uint8_t parentID;
    const char *text;
    uint16_t min;
    uint16_t max;
};

void dryStart();
void storageStart();
void autoPidM();
void saveAll();

// //!! НЕ УДАЛЯТЬ
// // const char f0[] EEMEM = "ПРОШИВКА";
// const char f0[] PROGMEM = "ПРОШИВКА";
// const char f1[] PROGMEM = "АЙДРАЙ";
// const char f2[] PROGMEM = "ВЕРСИЯ";
// const char f3[] PROGMEM = "0.1.0";
// const char f4[] PROGMEM = "ОХЛАЖДЕНИЕ";
// const char f5[] PROGMEM = "СОХРАНЕНЕНО";

// const char *const serviceTxt[] PROGMEM = {
//     f0,
//     f1,
//     f2,
//     f3,
//     f4,
//     f5,
// };

// //!! НЕ УДАЛЯТЬ

// const char s0[] PROGMEM = "ВОЗДУХ";
// const char s1[] PROGMEM = "НАГРЕВАТЕЛЬ";
// const char s2[] PROGMEM = "ВЛАЖНОСТЬ";
// const char s3[] PROGMEM = "ОШИБКА";
// const char s4[] PROGMEM = "ПРОВЕРЬ";
// const char s5[] PROGMEM = "ТЕРМИСТОР";
// const char s6[] PROGMEM = "ПЕРЕГУЗИ";

// const char *const txt[] PROGMEM = {
//     s0,
//     s1,
//     s2,
//     s3,
//     s4,
//     s5,
//     s6,
// };

const char s0[] PROGMEM = "ПРОШИВКА";
const char s1[] PROGMEM = "АЙДРАЙ";
const char s2[] PROGMEM = "ВРЕСИЯ";
const char s3[] PROGMEM = "0.1.0";
const char s4[] PROGMEM = "ОХЛАЖДЕНИЕ";
const char s5[] PROGMEM = "СОХРАНЕНО";
const char s6[] PROGMEM = "ВОЗДУХ";
const char s7[] PROGMEM = "НАГРЕВАТЕЛЬ";
const char s8[] PROGMEM = "ВЛАЖНОСТЬ";
const char s9[] PROGMEM = "ОШИБКА";
const char s10[] PROGMEM = "ПРОВЕРЬ";
const char s11[] PROGMEM = "ТЕРМИСТОР";
const char s12[] PROGMEM = "ПЕРЕГРУЗИ";

const char *const serviceTxt[] PROGMEM = {
    s0,
    s1,
    s2,
    s3,
    s4,
    s5,
    s6,
    s7,
    s8,
    s9,
    s10,
    s11,
    s12,
};

//!! НЕ УДАЛЯТЬ

const char i0[] PROGMEM = "МЕНЮ";
const char i1[] PROGMEM = "СУШКА";
const char i2[] PROGMEM = "ТЕМПЕРАТУРА";
const char i3[] PROGMEM = "ВРЕМЯ";
const char i4[] PROGMEM = "СТАРТ";
const char i5[] PROGMEM = "ХРАНЕНИЕ";
const char i6[] PROGMEM = "ТЕМПЕРАТУРА";
const char i7[] PROGMEM = "ВЛАЖНОСТЬ";
const char i8[] PROGMEM = "СТАРТ";
const char i9[] PROGMEM = "ПРЕСЕТЫ";
const char i10[] PROGMEM = "ПЛА";
const char i11[] PROGMEM = "ТЕМПЕРАТУРА";
const char i12[] PROGMEM = "ВРЕМЯ";
const char i13[] PROGMEM = "СТАРТ";
const char i14[] PROGMEM = "ПЕТГ";
const char i15[] PROGMEM = "ТЕМПЕРАТУРА";
const char i16[] PROGMEM = "ВРЕМЯ";
const char i17[] PROGMEM = "СТАРТ";
const char i18[] PROGMEM = "АБС";
const char i19[] PROGMEM = "ТЕМПЕРАТУРА";
const char i20[] PROGMEM = "ВРЕМЯ";
const char i21[] PROGMEM = "СТАРТ";
const char i22[] PROGMEM = "НАСТРОЙКИ";
const char i23[] PROGMEM = "ПИД";
const char i24[] PROGMEM = "КП";
const char i25[] PROGMEM = "КИ";
const char i26[] PROGMEM = "КД";
const char i27[] PROGMEM = "АВТОПИД";
const char i28[] PROGMEM = "ВРЕМЯ/МС";
const char i29[] PROGMEM = "ТЕМПЕРАТУРА";
const char i30[] PROGMEM = "СТАРТ";
const char i31[] PROGMEM = "ОБДУВ";
const char i32[] PROGMEM = "ДЕЛЬТА";
const char i33[] PROGMEM = "СЕРВО";
const char i34[] PROGMEM = "СЕРВО1";
const char i35[] PROGMEM = "ЗАКРЫТО";
const char i36[] PROGMEM = "ОТКРЫТО";
const char i37[] PROGMEM = "УГОЛ";
const char i38[] PROGMEM = "СЕРВО2";
const char i39[] PROGMEM = "ЗАКРЫТО";
const char i40[] PROGMEM = "ОТКРЫТО";
const char i41[] PROGMEM = "УГОЛ";
const char i42[] PROGMEM = "СОХРАНИТЬ";

const menuS menuPGM[] PROGMEM = {
    {0, NULL, i0, 0, 0},
    {1, 0, i1, 0, 0},
    {2, 1, i2, 0, 100},
    {3, 1, i3, 0, 600},
    {4, 1, i4, 0, 0},
    {5, 0, i5, 0, 0},
    {6, 5, i6, 0, 100},
    {7, 5, i7, 5, 40},
    {8, 5, i8, 0, 0},
    {9, 0, i9, 0, 0},
    {10, 9, i10, 0, 0},
    {11, 10, i11, 45, 100},
    {12, 10, i12, 120, 600},
    {13, 10, i13, 0, 0},
    {14, 9, i14, 0, 0},
    {15, 14, i15, 45, 100},
    {16, 14, i16, 120, 600},
    {17, 14, i17, 0, 0},
    {18, 9, i18, 0, 0},
    {19, 18, i19, 45, 100},
    {20, 18, i20, 120, 600},
    {21, 18, i21, 0, 0},
    {22, 0, i22, 0, 0},
    {23, 22, i23, 0, 0},
    {24, 23, i24, 0, 10000},
    {25, 23, i25, 0, 10000},
    {26, 23, i26, 0, 10000},
    {27, 23, i27, 0, 0},
    {28, 27, i28, 200, 10000},
    {29, 27, i29, 30, 100},
    {30, 27, i30, 0, 0},
    {31, 22, i31, 20, 100},
    {32, 22, i32, 0, 15},
    {33, 22, i33, 0, 0},
    {34, 33, i34, 0, 0},
    {35, 34, i35, 0, 60},
    {36, 34, i36, 0, 10},
    {37, 34, i37, 0, 90},
    {38, 33, i38, 0, 0},
    {39, 38, i39, 0, 60},
    {40, 38, i40, 0, 10},
    {41, 38, i41, 0, 90},
    {42, 22, i42, 0, 0},
};

uint16_t menuVal[] =
    {
        123,
        0,
        60,
        240,
        0,
        0,
        35,
        15,
        0,
        0,
        0,
        50,
        240,
        0,
        0,
        50,
        240,
        0,
        0,
        50,
        240,
        0,
        0,
        0,
        400,
        200,
        200,
        0,
        500,
        60,
        0,
        50,
        8,
        0,
        0,
        30,
        1,
        60,
        0,
        30,
        1,
        60,
        0,
};

typedef void (*ptrFunc)();

const ptrFunc menuFunc[]{
    NULL,
    NULL,
    NULL,
    NULL,
    &dryStart,
    NULL,
    NULL,
    NULL,
    &storageStart,
    NULL,
    NULL,
    NULL,
    NULL,
    &dryStart,
    NULL,
    NULL,
    NULL,
    &dryStart,
    NULL,
    NULL,
    NULL,
    &dryStart,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    &autoPidM,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    &saveAll,
};