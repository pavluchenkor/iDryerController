#include <Arduino.h>
#include <Configuration.h>

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

uint16_t EEMEM eToEemem  = 1;

// #ifdef WITH_BLACKJACK_AND_HOOKERS
// uint16_t eToEemem EEMEM = 0b0000000000000000;
// #else
// uint16_t eToEemem = 0b0000000000000000;
// #endif

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

const char EEMEM s0[]  = "ПРОШИВКА";
const char EEMEM s1[]  = "АЙДРАЙ";
const char EEMEM s2[]  = "ВРЕСИЯ";
const char EEMEM s3[]  = "0.1.0";
const char EEMEM s4[]  = "ОХЛАЖДЕНИЕ";
const char EEMEM s5[]  = "СОХРАНЕНО";
const char EEMEM s6[]  = "ВОЗДУХ";
const char EEMEM s7[]  = "НАГРЕВАТЕЛЬ";
const char EEMEM s8[]  = "ВЛАЖНОСТЬ";
const char EEMEM s9[]  = "ОШИБКА";
const char EEMEM s10[]  = "ПРОВЕРЬ";
const char EEMEM s11[]  = "ТЕРМИСТОР";
const char EEMEM s12[]  = "ПЕРЕГРУЗИ";

const char *const EEMEM serviceTxt[]  = {
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

const char EEMEM i0[] = "МЕНЮ";
const char EEMEM i1[] = "СУШКА";
const char EEMEM i2[] = "ТЕМПЕРАТУРА";
const char EEMEM i3[] = "ВРЕМЯ";
const char EEMEM i4[] = "СТАРТ";
const char EEMEM i5[] = "ХРАНЕНИЕ";
const char EEMEM i6[] = "ТЕМПЕРАТУРА";
const char EEMEM i7[] = "ВЛАЖНОСТЬ";
const char EEMEM i8[] = "СТАРТ";
const char EEMEM i9[] = "ПРЕСЕТЫ";
const char EEMEM i10[] = "ПЛА";
const char EEMEM i11[] = "ТЕМПЕРАТУРА";
const char EEMEM i12[] = "ВРЕМЯ";
const char EEMEM i13[] = "СТАРТ";
const char EEMEM i14[] = "ПЕТГ";
const char EEMEM i15[] = "ТЕМПЕРАТУРА";
const char EEMEM i16[] = "ВРЕМЯ";
const char EEMEM i17[] = "СТАРТ";
const char EEMEM i18[] = "АБС";
const char EEMEM i19[] = "ТЕМПЕРАТУРА";
const char EEMEM i20[] = "ВРЕМЯ";
const char EEMEM i21[] = "СТАРТ";
const char EEMEM i22[] = "НАСТРОЙКИ";
const char EEMEM i23[] = "ПИД";
const char EEMEM i24[] = "КП";
const char EEMEM i25[] = "КИ";
const char EEMEM i26[] = "КД";
const char EEMEM i27[] = "АВТОПИД";
const char EEMEM i28[] = "ВРЕМЯ/МС";
const char EEMEM i29[] = "ТЕМПЕРАТУРА";
const char EEMEM i30[] = "СТАРТ";
const char EEMEM i31[] = "ОБДУВ";
const char EEMEM i32[] = "ДЕЛЬТА";
const char EEMEM i33[] = "СЕРВО";
const char EEMEM i34[] = "СЕРВО1";
const char EEMEM i35[] = "ЗАКРЫТО";
const char EEMEM i36[] = "ОТКРЫТО";
const char EEMEM i37[] = "УГОЛ";
const char EEMEM i38[] = "СЕРВО2";
const char EEMEM i39[] = "ЗАКРЫТО";
const char EEMEM i40[] = "ОТКРЫТО";
const char EEMEM i41[] = "УГОЛ";
const char EEMEM i42[] = "СОХРАНИТЬ";

const menuS EEMEM menuPGM[]  = {
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
    {41, 38, i41, 0, 80},
    {42, 22, i42, 0, 0},
};

uint16_t EEMEM menuVal[] = {
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