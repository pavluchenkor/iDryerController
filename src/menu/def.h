#define DEF_T_FIRMWARE 0
#define DEF_T_AYRA 1
#define DEF_T_VERSION 2
#define DEF_T_VER 3
#define DEF_T_COOLING 4
#define DEF_T_SAVED 5
#define DEF_T_AIR 6
#define DEF_T_HEATER 7
#define DEF_T_HUMIDITY 8
#define DEF_T_ERROR 9
#define DEF_T_CHECK 10
#define DEF_T_THERMISTOR 11
#define DEF_T_OVERLOAD 12
#define DEF_T_REMOVE 13
#define DEF_T_SPOOL 14
#define DEF_T_SETUP 15
#define DEF_T_PUT 16
#define DEF_T_WEIGHT 17
#define DEF_T_GRAM 18
#define DEF_T_CELSIUS 19
#define DEF_T_BURN 20
#define DEF_T_PART 21
#define DEF_T_FREQUENCY 22

#define DEF_MENU_DRYING 1
#define DEF_DRYING_TEMPERATURE 2
#define DEF_DRYING_TIME 3
#define DEF_DRYING_START 4
#define DEF_MENU_STORAGE 5
#define DEF_STORAGE_TEMPERATURE 6
#define DEF_STORAGE_HUMIDITY 7
#define DEF_STORAGE_START 8
#define DEF_MENU_PRESETS 9
#define DEF_PRESETS_PLA 10
#define DEF_PLA_TEMPERATURE 11
#define DEF_PLA_TIME 12
#define DEF_PLA_START 13
#define DEF_PRESETS_PETG 14
#define DEF_PETG_TEMPERATURE 15
#define DEF_PETG_TIME 16
#define DEF_PETG_START 17
#define DEF_PRESETS_ABS 18
#define DEF_ABS_TEMPERATURE 19
#define DEF_ABS_TIME 20
#define DEF_ABS_START 21
#define DEF_MENU_SETTINGS 22
#define DEF_SETTINGS_PID 23
#define DEF_PID_KP 24
#define DEF_PID_KI 25
#define DEF_PID_KD 26
#define DEF_PID_AVTOPID 27
#define DEF_AVTOPID_TIME_MS 28
#define DEF_AVTOPID_TEMPERATURE 29
#define DEF_AVTOPID_START 30
#define DEF_SETTINGS_BLOWING 31
#define DEF_SETTINGS_DELTA 32
#define DEF_SETTINGS_SERVO 33
#define DEF_SERVO_CLOSED 34
#define DEF_SERVO_OPEN 35
#define DEF_SERVO_CORNER 36
#define DEF_SERVO_TEST 37
#define DEF_SETTINGS_SAVE 38

#if SCALES_MODULE_NUM > 0
#define DEF_MENU_SCALES 39
#define DEF_SCALES_COIL_1 40
#define DEF_COIL_1_TARA 41
#define DEF_COIL_1_INSTALLATION 42
#endif

#if SCALES_MODULE_NUM > 1
#define DEF_SCALES_COIL_2 43
#define DEF_COIL_2_TARA 44
#define DEF_COIL_2_INSTALLATION 45
#endif

#if SCALES_MODULE_NUM > 2
#define DEF_SCALES_COIL_3 46
#define DEF_COIL_3_TARA 47
#define DEF_COIL_3_INSTALLATION 48
#endif

#if SCALES_MODULE_NUM > 3
#define DEF_SCALES_COIL_4 49
#define DEF_COIL_4_TARA 50
#define DEF_COIL_4_INSTALLATION 51
#endif