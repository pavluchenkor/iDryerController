#include <Arduino.h>
#include "Configuration.h"

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

#define EEPROM_VALIDATION_VALUE 123

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
void autoPidStart();
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

PGM_P const menuTxt[] PROGMEM = {
    i0,  // id: 0
    i1,  // id: 1
    i2,  // id: 2
    i3,  // id: 3
    i4,  // id: 4
    i5,  // id: 5
    i2,  // id: 6
    i7,  // id: 7
    i4,  // id: 8
    i9,  // id: 9
    i10, // id: 10
    i2,  // id: 11
    i3,  // id: 12
    i4,  // id: 13
    i14, // id: 14
    i2,  // id: 15
    i3,  // id: 16
    i4,  // id: 17
    i18, // id: 18
    i2,  // id: 19
    i3,  // id: 20
    i4,  // id: 21
    i22, // id: 22
    i23, // id: 23
    i24, // id: 24
    i25, // id: 25
    i26, // id: 26
    i27, // id: 27
    i28, // id: 28
    i29, // id: 29
    i2,  // id: 30
    i4,  // id: 31
    i31, // id: 32
    i32, // id: 33
    i33, // id: 34
    i34, // id: 35
    i35, // id: 36
    i36, // id: 37
    i37, // id: 38
    i38, // id: 39
#if SCALES_MODULE_NUM > 0
    i39, // id: 40
    i40, // id: 41
    i41, // id: 42
    i42, // id: 43
#endif
#if SCALES_MODULE_NUM > 1
    i43, // id: 44
    i41, // id: 45
    i42, // id: 46
#endif
#if SCALES_MODULE_NUM > 2
    i46, // id: 47
    i41, // id: 48
    i42, // id: 49
#endif
#if SCALES_MODULE_NUM > 3
    i49, // id: 50
    i41, // id: 51
    i42, // id: 52
#endif
};

const menuS menuPGM[] PROGMEM = {
    {0, NULL, 0, 0}, // id: 0
    {1, 0, 0, 0},    // id: 1
#if CE == 100
    {2, 1, 0, 100}, // id: 2
#elif CE == 110
    {2, 1, 0, 110}, // id: 2
#endif
    {3, 1, 0, 1140}, // id: 3
    {4, 1, 0, 0},    // id: 4
    {5, 0, 0, 0},    // id: 5
#if CE == 100
    {6, 5, 0, 100}, // id: 6
#elif CE == 110
    {6, 5, 0, 110}, // id: 6
#endif
    {7, 5, 5, 40}, // id: 7
    {8, 5, 0, 0},  // id: 8
    {9, 0, 0, 0},  // id: 9
    {10, 9, 0, 0}, // id: 10
#if CE == 100
    {11, 10, 45, 100}, // id: 11
#elif CE == 110
    {11, 10, 45, 110}, // id: 11
#endif
    {12, 10, 120, 1140}, // id: 12
    {13, 10, 0, 0},      // id: 13
    {14, 9, 0, 0},       // id: 14
    {15, 14, 45, 100},   // id: 15
    {16, 14, 120, 1140}, // id: 16
    {17, 14, 0, 0},      // id: 17
    {18, 9, 0, 0},       // id: 18
#if CE == 100
    {19, 18, 45, 100}, // id: 19
#elif CE == 110
    {19, 18, 45, 110}, // id: 19
#endif
    {20, 18, 120, 1140}, // id: 20
    {21, 18, 0, 0},      // id: 21
    {22, 0, 0, 0},       // id: 22
    {23, 22, 0, 0},      // id: 23
    {24, 23, 0, 65535},  // id: 24
    {25, 23, 0, 65535},  // id: 25
    {26, 23, 0, 65535},  // id: 26
    {27, 23, 0, 65535},  // id: 27
    {28, 23, 5, 65535}, // id: 28
    {29, 23, 0, 0},      // id: 29
#if CE == 100
    {30, 29, 30, 100}, // id: 30
#elif CE == 110
    {30, 29, 30, 110}, // id: 30
#endif
    {31, 29, 0, 0},    // id: 31
    {32, 22, 70, 100}, // id: 32
    {33, 22, 0, 50},   // id: 33
    {34, 22, 0, 0},    // id: 34
    {35, 34, 0, 60},   // id: 35
    {36, 34, 0, 10},   // id: 36
    {37, 34, 0, 90},   // id: 37
    {38, 34, 0, 0},    // id: 38
    {39, 22, 0, 0},    // id: 39
#if SCALES_MODULE_NUM > 0
    {40, 0, 0, 0},    // id: 40
    {41, 40, 0, 0},   // id: 41
    {42, 41, 0, 500}, // id: 42
    {43, 41, 0, 0},   // id: 43
#endif
#if SCALES_MODULE_NUM > 1
    {44, 40, 0, 0},   // id: 44
    {45, 44, 0, 500}, // id: 45
    {46, 44, 0, 0},   // id: 46
#endif
#if SCALES_MODULE_NUM > 2
    {47, 40, 0, 0},   // id: 47
    {48, 47, 0, 500}, // id: 48
    {49, 47, 0, 0},   // id: 49
#endif
#if SCALES_MODULE_NUM > 3
    {50, 40, 0, 0},   // id: 50
    {51, 50, 0, 500}, // id: 51
    {52, 50, 0, 0},   // id: 52
#endif
};

uint16_t menuVal[] EEMEM = {
    EEPROM_VALIDATION_VALUE, // id: 0
    0,                       // id: 1
    60,                      // id: 2
    240,                     // id: 3
    0,                       // id: 4
    0,                       // id: 5
    35,                      // id: 6
    15,                      // id: 7
    0,                       // id: 8
    0,                       // id: 9
    0,                       // id: 10
    55,                      // id: 11
    180,                     // id: 12
    0,                       // id: 13
    0,                       // id: 14
    65,                      // id: 15
    240,                     // id: 16
    0,                       // id: 17
    0,                       // id: 18
    90,                      // id: 19
    240,                     // id: 20
    0,                       // id: 21
    0,                       // id: 22
    0,                       // id: 23
    10,                      // id: 24 Kp div100 (DEF_PID_KP_DIV)
    10,                      // id: 25 Ki div1000 (DEF_PID_KI_DIV)
    200,                     // id: 26 Kd div100 (DEF_PID_KD_DIV)
    50,                      // id: 27 Kf div100 (DEF_PID_KF_DIV)
    50,                      // id: 28 min pid delta time div100 (DEF_MIN_PID_DELTA_TIME_MS_DIV)
    0,                       // id: 29 autopid
    65,                      // id: 30 autopid temp
    0,                       // id: 31 autopid start
    100,                     // id: 32 airflow
    0,                       // id: 33 delta
    0,                       // id: 34
    30,                      // id: 35
    1,                       // id: 36
    55,                      // id: 37
    0,                       // id: 38
    0,                       // id: 39
#if SCALES_MODULE_NUM > 0
    0, // id: 40
    0, // id: 41
    0, // id: 42
    0, // id: 43
#endif
#if SCALES_MODULE_NUM > 1
    0, // id: 44
    0, // id: 45
    0, // id: 46
#endif
#if SCALES_MODULE_NUM > 2
    0, // id: 47
    0, // id: 48
    0, // id: 49
#endif
#if SCALES_MODULE_NUM > 3
    0, // id: 50
    0, // id: 51
    0, // id: 52
#endif
};

typedef void (*ptrFunc)();

const ptrFunc menuFunc[]{
    NULL,          // id: 0
    NULL,          // id: 1
    NULL,          // id: 2
    NULL,          // id: 3
    &mainDryStart, // id: 4
    NULL,          // id: 5
    NULL,          // id: 6
    NULL,          // id: 7
    &storageStart, // id: 8
    NULL,          // id: 9
    NULL,          // id: 10
    NULL,          // id: 11
    NULL,          // id: 12
    &plaDryStart,  // id: 13
    NULL,          // id: 14
    NULL,          // id: 15
    NULL,          // id: 16
    &petgDryStart, // id: 17
    NULL,          // id: 18
    NULL,          // id: 19
    NULL,          // id: 20
    &absDryStart,  // id: 21
    NULL,          // id: 22
    NULL,          // id: 23
    NULL,          // id: 24
    NULL,          // id: 25
    NULL,          // id: 26
    NULL,          // id: 27
    NULL,          // id: 28
    NULL,          // id: 29
    NULL,          // id: 30
    &autoPidStart, // id: 31
    NULL,          // id: 32
    NULL,          // id: 33
    NULL,          // id: 34
    NULL,          // id: 35
    NULL,          // id: 36
    NULL,          // id: 37
    &servoTest,    // id: 38
    &saveAll,      // id: 39
#if SCALES_MODULE_NUM > 0 && AUTOPID_RUN == 0
    NULL,      // id: 40
    NULL,      // id: 41
    NULL,      // id: 42
    setSpool1, // id: 43
#endif
#if SCALES_MODULE_NUM > 1 && AUTOPID_RUN == 0
    NULL,       // id: 44
    NULL,       // id: 45
    &setSpool2, // id: 46
#endif
#if SCALES_MODULE_NUM > 2 && AUTOPID_RUN == 0
    NULL,       // id: 47
    NULL,       // id: 48
    &setSpool3, // id: 49
#endif
#if SCALES_MODULE_NUM > 3 && AUTOPID_RUN == 0
    NULL,       // id: 50
    NULL,       // id: 51
    &setSpool4, // id: 52
#endif
};