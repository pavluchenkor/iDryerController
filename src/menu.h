#include <Arduino.h>
// #include <main.cpp>

struct menuS
{
  uint8_t id;
  uint8_t parentID;
  uint8_t level;
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
const char i2[] PROGMEM = "ТЕМП.";
const char i3[] PROGMEM = "ВРЕМЯ";
const char i4[] PROGMEM = "СТАРТ";
const char i5[] PROGMEM = "ХРАНЕНИЕ";
const char i6[] PROGMEM = "ТЕМП.";
const char i7[] PROGMEM = "ВЛАЖНОСТЬ";
const char i8[] PROGMEM = "СТАРТ";
const char i9[] PROGMEM = "ПРЕСЕТЫ";
const char i10[] PROGMEM = "ПЛА";
const char i11[] PROGMEM = "ТЕМП.";
const char i12[] PROGMEM = "ВРЕМЯ";
const char i13[] PROGMEM = "СТАРТ";
const char i14[] PROGMEM = "ПЕТГ";
const char i15[] PROGMEM = "ТЕМП.";
const char i16[] PROGMEM = "ВРЕМЯ";
const char i17[] PROGMEM = "СТАРТ";
const char i18[] PROGMEM = "АБС";
const char i19[] PROGMEM = "ТЕМП.";
const char i20[] PROGMEM = "ВРЕМЯ";
const char i21[] PROGMEM = "СТАРТ";
const char i22[] PROGMEM = "НАСТРОЙКИ";
const char i23[] PROGMEM = "АВТОПИД";
const char i24[] PROGMEM = "ОБДУВ";
const char i25[] PROGMEM = "СОХРАНИТЬ";

const menuS menuPGM[] PROGMEM = {
    {0, NULL, 0, i0, 0, 0},
    {1, 0, 1, i1, 0, 0},
    {2, 1, 2, i2, 0, 100},
    {3, 1, 2, i3, 0, 600},
    {4, 1, 2, i4, 0, 0},
    {5, 0, 1, i5, 0, 0},
    {6, 5, 2, i6, 0, 100},
    {7, 5, 2, i7, 5, 40},
    {8, 5, 2, i8, 0, 0},
    {9, 0, 1, i9, 0, 0},
    {10, 9, 2, i10, 0, 0},
    {11, 10, 3, i11, 45, 100},
    {12, 10, 3, i12, 120, 600},
    {13, 10, 3, i13, 0, 0},
    {14, 9, 2, i14, 0, 0},
    {15, 14, 3, i15, 45, 100},
    {16, 14, 3, i16, 120, 600},
    {17, 14, 3, i17, 0, 0},
    {18, 9, 2, i18, 0, 0},
    {19, 18, 3, i19, 45, 100},
    {20, 18, 3, i20, 120, 600},
    {21, 18, 3, i21, 0, 0},
    {22, 0, 1, i22, 0, 0},
    {23, 22, 2, i23, 0, 0},
    {24, 22, 2, i24, 20, 100},
    {25, 22, 2, i25, 0, 0},
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
        50,
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
    &autoPidM,
    NULL,
    &saveAll,
};