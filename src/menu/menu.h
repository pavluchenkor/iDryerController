#include <Arduino.h>
#include "Configuration.h"

#ifdef WITH_BLACKJACK_AND_HOOKERS
#if LANG == 0
#include "menu_ru_ext.h"
#include "font_ru.h"
#define PADDING_V 0
#define PADDING_OFFSET 0
#elif LANG == 1
#include "menu_en_ext.h"
#include "font_en.h"
#define PADDING_OFFSET 1
#define PADDING_V 2
#endif
#else
#if LANG == 0
#include "menu_ru.h"
#elif LANG == 1
#include "menu_no_rus.h"
#endif
#endif

struct menuS
{
    uint8_t id;
    uint8_t parentID;
    // const char *text;
    uint16_t min;
    uint16_t max;
};

void dryStart();
void mainDryStart();
void plaDryStart();
void petgDryStart();
void absDryStart();
void storageStart();
void autoPidM();
void saveAll();
void servoTest();
#if SCALES_MODULE_NUM > 0
void setSpool1();
#endif
#if SCALES_MODULE_NUM > 1
void setSpool2();
#endif
#if SCALES_MODULE_NUM > 2
void setSpool3();
#endif
#if SCALES_MODULE_NUM > 3
void setSpool4();
#endif

PGM_P const serviceTxt[] PROGMEM = {
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
    s13,
    s14,
    s15,
    s16,
    s17,
    s18,
    s19,
    s20,
    s21,
    s22,
};

#ifdef WITH_BLACKJACK_AND_HOOKERS
PGM_P const menuTxt[] PROGMEM = {
    i0,
    i1,
    i2,
    i3,
    i4,
    i5,
    i2,
    i7,
    i4,
    i9,
    i10,
    i2,
    i3,
    i4,
    i14,
    i2,
    i3,
    i4,
    i18,
    i2,
    i3,
    i4,
    i22,
    i23,
    i24,
    i25,
    i26,
    i27,
    i28,
    i2,
    i4,
    i31,
    i32,
    i33,
    i34,
    i35,
    i36,
    i37,
    i38,
#if SCALES_MODULE_NUM > 0
    i39,
    i40,
    i41,
    i42,
#endif
#if SCALES_MODULE_NUM > 1
    i43,
    i41,
    i42,
#endif
#if SCALES_MODULE_NUM > 2
    i46,
    i41,
    i42,
#endif
#if SCALES_MODULE_NUM > 3
    i49,
    i41,
    i42,
#endif
};
#else
PGM_P const menuTxt[] PROGMEM = {
    i0,
    i1,
    i2,
    i3,
    i4,
    i5,
    i2,
    i7,
    i4,
    i9,
    i10,
    i2,
    i3,
    i4,
    i14,
    i2,
    i3,
    i4,
    i18,
    i2,
    i3,
    i4,
    i22,
    i23,
    i24,
    i25,
    i26,
    i27,
    i28,
    i2,
    i4,
    i31,
    i32,
    i33,
};
#endif

// uint16_t menuVal[] EEMEM =
const menuS menuPGM[] PROGMEM = {
    {0, NULL, 0, 0},
    {1, 0, 0, 0},
    {2, 1, 0, 100},
    {3, 1, 0, 600},
    {4, 1, 0, 0},
    {5, 0, 0, 0},
    {6, 5, 0, 100},
    {7, 5, 5, 40},
    {8, 5, 0, 0},
    {9, 0, 0, 0},
    {10, 9, 0, 0},
    {11, 10, 45, 100},
    {12, 10, 120, 600},
    {13, 10, 0, 0},
    {14, 9, 0, 0},
    {15, 14, 45, 100},
    {16, 14, 120, 600},
    {17, 14, 0, 0},
    {18, 9, 0, 0},
    {19, 18, 45, 100},
    {20, 18, 120, 600},
    {21, 18, 0, 0},
    {22, 0, 0, 0},
    {23, 22, 0, 0},
    {24, 23, 0, 10000},
    {25, 23, 0, 10000},
    {26, 23, 0, 10000},
    {27, 23, 0, 0},
    {28, 27, 50, 1000},
    {29, 27, 30, 100},
    {30, 27, 0, 0},
    {31, 22, 20, 100},
    {32, 22, 0, 50},
    {33, 22, 0, 0},
    {34, 33, 0, 60},
    {35, 33, 0, 10},
    {36, 33, 0, 90},
    {37, 33, 0, 0},
    {38, 22, 0, 0},
#if SCALES_MODULE_NUM > 0
    {39, 0, 0, 0},
    {40, 39, 0, 0},
    {41, 40, 0, 500},
    {42, 40, 0, 0},
#endif
#if SCALES_MODULE_NUM > 1
    {43, 39, 0, 0},
    {44, 43, 0, 500},
    {45, 43, 0, 0},
#endif
#if SCALES_MODULE_NUM > 2
    {46, 39, 0, 0},
    {47, 46, 0, 500},
    {48, 46, 0, 0},
#endif
#if SCALES_MODULE_NUM > 3
    {49, 39, 0, 0},
    {50, 49, 0, 500},
    {51, 49, 0, 0},
#endif
};

#ifdef WITH_BLACKJACK_AND_HOOKERS

#else
uint16_t menuValDefault[] = {
    123, // id: 0
    0,   // id: 1
    60,  // id: 2
    240, // id: 3
    0,   // id: 4
    0,   // id: 5
    35,  // id: 6
    15,  // id: 7
    0,   // id: 8
    0,   // id: 9
    0,   // id: 10
    50,  // id: 11
    240, // id: 12
    0,   // id: 13
    0,   // id: 14
    50,  // id: 15
    240, // id: 16
    0,   // id: 17
    0,   // id: 18
    50,  // id: 19
    240, // id: 20
    0,   // id: 21
    0,   // id: 22
    0,   // id: 23
    400, // id: 24
    200, // id: 25
    200, // id: 26
    0,   // id: 27
    500, // id: 28
    60,  // id: 29
    0,   // id: 30
    50,  // id: 31
    8,   // id: 32
    0,   // id: 33
    0,   // id: 34
    30,  // id: 35
    1,   // id: 36
    60,  // id: 37
    0,   // id: 38
    30,  // id: 39
    1,   // id: 40
    60,  // id: 41
    0,   // id: 42
};
#endif

uint16_t menuVal[] EEMEM = {
    123, // id: 0
    0,   // id: 1
    60,  // id: 2
    240, // id: 3
    0,   // id: 4
    0,   // id: 5
    35,  // id: 6
    15,  // id: 7
    0,   // id: 8
    0,   // id: 9
    0,   // id: 10
    55,  // id: 11
    180, // id: 12
    0,   // id: 13
    0,   // id: 14
    65,  // id: 15
    240, // id: 16
    0,   // id: 17
    0,   // id: 18
    90,  // id: 19
    240, // id: 20
    0,   // id: 21
    0,   // id: 22
    0,   // id: 23
    500, // id: 24
    1,   // id: 25
    200, // id: 26
    0,   // id: 27
    400, // id: 28
    75,  // id: 29
    0,   // id: 30
    75,  // id: 31
    8,   // id: 32
    0,   // id: 33
    30,  // id: 34
    1,   // id: 35
    55,  // id: 36
    0,   // id: 37
    0,   // id: 38
#if SCALES_MODULE_NUM > 0
    0, // id: 39
    0, // id: 40
    0, // id: 41
    0, // id: 42
#endif
#if SCALES_MODULE_NUM > 1
    0, // id: 43
    0, // id: 44
    0, // id: 45
#endif
#if SCALES_MODULE_NUM > 2
    0, // id: 46
    0, // id: 47
    0, // id: 48
#endif
#if SCALES_MODULE_NUM > 3
    0, // id: 49
    0, // id: 50
    0, // id: 51
#endif
};

typedef void (*ptrFunc)();

const ptrFunc menuFunc[]
{
    NULL,
        NULL,
        NULL,
        NULL,
        &mainDryStart,
        NULL,
        NULL,
        NULL,
        &storageStart,
        NULL,
        NULL,
        NULL,
        NULL,
        &plaDryStart,
        NULL,
        NULL,
        NULL,
        &petgDryStart,
        NULL,
        NULL,
        NULL,
        &absDryStart,
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
        &servoTest,
        &saveAll,
#if SCALES_MODULE_NUM > 0 && AUTOPID_RUN == 0
        NULL,
        NULL,
        NULL,
        setSpool1,
#endif
#if SCALES_MODULE_NUM > 1 && AUTOPID_RUN == 0
        NULL,
        NULL,
        &setSpool2,
#endif
#if SCALES_MODULE_NUM > 2 && AUTOPID_RUN == 0
        NULL,
        NULL,
        &setSpool3,
#endif
#if SCALES_MODULE_NUM > 3 && AUTOPID_RUN == 0
        NULL,
        NULL,
        &setSpool4,
#endif
};