#include <Arduino.h>
// #include <main.cpp>

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

//!! НЕ УДАЛЯТЬ
const char s0[] PROGMEM = "СУШКА";
const char s1[] PROGMEM = "ХРАНЕНИЕ";
const char l0[] PROGMEM = "ВОЗДУХ";
const char l1[] PROGMEM = "НАГРЕВАТЕЛЬ";
const char l2[] PROGMEM = "ВЛАЖНОСТЬ";
const char l3[] PROGMEM = "ОШИБКА";
const char l4[] PROGMEM = "ПРОВЕРЬ";
const char l5[] PROGMEM = "ТЕРМИСТОР";
const char l6[] PROGMEM = "ПЕРЕГУЗИ";

const char *const txt[] PROGMEM = {
    s0,
    s1,
    l0,
    l1,
    l2,
    l3,
    l4,
    l5,
    l6,
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
const char i33[] PROGMEM = "СОХРАНИТЬ";

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
    &saveAll,
};