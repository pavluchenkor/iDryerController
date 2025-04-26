#include <Arduino.h>
#include <Configuration.h>
#include <Wire.h>
#if SCALES_MODULE_NUM != 0 && AUTOPID_RUN == 1
#include <EEPROM.h>
#endif
#include <avr/wdt.h>
#include <U8g2lib.h>
#include <GyverTimers.h>

#if SENSOR_TYPE == 0
#define ERROR
#elif SENSOR_TYPE == 1
#define SENSOR_BME280
#elif SENSOR_TYPE == 2
#define SENSOR_SHT31
#endif

#ifdef SENSOR_BME280
  #include <GyverBME280.h>
#endif

#ifdef SENSOR_SHT31
  #include <SHT31.h>
#endif


#include <PID_v1.h>
#include <pid/pidautotuner.h> //https://github.com/jackw01/arduino-pid-autotuner
#include <thermistor/thermistor.h>
#include "menu/menu.h"
#include "menu/def.h"
#include "math/math_extensions.h"
#include "servo/servo.h"
#include "buzzer/buzzer.h"
#include "HX711.h"
#include <EncButton.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// #define KASYAK_FINDER
#ifdef KASYAK_FINDER
#define DEBUG_PRINT(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#endif

// Пороговые значения для температурных фаз
#define HEATING_THRESHOLD 10.0f // Порог для агрессивного нагрева (°C)
#define HEATER_AIR_DELTA 0.5f   // Компенсация теплопотерь (°C)
#define CRITICAL_OVERHEAT 5.0f  // Критическая температура (°C)

// #define DEBUG
#ifdef DEBUG
uint8_t testPWM = 0;
uint32_t testTIMER_COUNT = 0;
uint16_t testTIMER_STATE = 0;
unsigned long oldTime1 = 0;
unsigned long oldTime2 = 0;
unsigned long oldTime3 = 0;
unsigned long printOldTime = 0;
uint8_t isrFlag = 0;
#endif

uint32_t ERROR_CODE EEMEM = 0x0;

#if REV == 0
#define ERROR
#elif REV == 1
#define v220V
#elif REV == 2
#define v24V
#endif

#if PWM_11_FREQUENCY == 62500
#define TCCR2B_PRESCALER 0b00000001
#define TCCR2A_MODE 0b00000011
#elif PWM_11_FREQUENCY == 31400
#define TCCR2B_PRESCALER 0b00000001
#define TCCR2A_MODE 0b00000001
#elif PWM_11_FREQUENCY == 8000
#define TCCR2B_PRESCALER 0b00000010
#define TCCR2A_MODE 0b00000011
#elif PWM_11_FREQUENCY == 4000
#define TCCR2B_PRESCALER 0b00000010
#define TCCR2A_MODE 0b00000001
#elif PWM_11_FREQUENCY == 2000
#define TCCR2B_PRESCALER 0b00000011
#define TCCR2A_MODE 0b00000011
#elif PWM_11_FREQUENCY == 980
#define TCCR2B_PRESCALER 0b00000011
#define TCCR2A_MODE 0b00000001
#elif PWM_11_FREQUENCY == 245
#define TCCR2B_PRESCALER 0b00000101
#define TCCR2A_MODE 0b00000001
#elif PWM_11_FREQUENCY == 122
#define TCCR2B_PRESCALER 0b00000110
#define TCCR2A_MODE 0b00000001
#elif PWM_11_FREQUENCY == 60
#define TCCR2B_PRESCALER 0b00000111
#define TCCR2A_MODE 0b00000011
#elif PWM_11_FREQUENCY == 30
#define TCCR2B_PRESCALER 0b00000111
#define TCCR2A_MODE 0b00000001
#else
#define TCCR2B_PRESCALER 0b00000100
#define TCCR2A_MODE 0b00000001
#endif

#if REV == 0
#define ERROR
#elif REV == 1
#define v220V
#elif REV == 2
#define v24V
#endif

#ifdef v220V
/*********************
 * НЕ ВЛЕЗАЙ! УЕБЬЕТ!
 * меняй это на свой страх и риск
 ********************/
#define HEATER_MIN 500
#define HEATER_MAX 2000
#define HEATER_OFF 9500
#endif

#ifdef v24V
/**********************
 * 0-255 PWM 8bit
 * If your heater is very powerful,
 * for example more than 100-200 watts,
 * this can be reduced to 200-150
 ********************/
#define HEATER_MIN 0
#define HEATER_MAX 255
#endif

#define NTC_PIN 0
#ifdef v220V
#define ZERO_PIN 2
#define INT_NUM 0
#endif
#define DIMMER_PIN 5

#define BUZZER_PIN 3
#define FAN 11 // TODO EXT обновить инструкцию
#define SERVO_1_PIN 7

#if SCALES_MODULE_NUM != 0
#define FILAMENT_SENSOR 4
#endif

#define SCREEN_LINES 4
#define MENU_HEADER 1

enum stateS
{
    OFF,
    ON,
    MENU,
    DRY,
    STORAGE,
    AUTOPID,
    NTC_ERROR,
};

#define MAX_ERROR 30

#define ADC_MIN 200
#define ADC_MAX 1000
uint16_t tmpTemp = (ADC_MIN + ADC_MAX) / 2;

#define LINE_HIGHT 16
uint16_t dimmer = 0;

#ifdef v220V
uint16_t lastDim;
#endif

stateS state = MENU;
uint8_t ERROR_COUNTER = 0;

uint8_t globalErrorFuncUUID = 0;

const byte CLK = A1;
const byte DT = A2;
const byte encBut = A3;
volatile static byte PCMask;
volatile byte flagISR = 0;

bool timer1_dimmerFlag = false;
uint8_t timer2_aFlag = 0;
uint8_t menuSize = 0;
unsigned long oldTime = 0;
unsigned long oldTimer = 0;
unsigned long scaleTimer = 0;
boolean isScaleShow = true;

unsigned long screenTime = 0;
char serviceString[30];

enum levelV
{
    NOTHING,
    DOWN,
    UP,
};

struct subMenu
{
    bool changeVal;
    bool pointerUpdate;
    levelV levelUpdate = NOTHING;
    uint8_t level; // текущий уровень меню
    uint8_t levelMax;
    int8_t parentID;
    uint8_t membersQuantity;
    uint8_t membersID[25];
    uint8_t linesToScreen[SCREEN_LINES - MENU_HEADER];
    int8_t position;
    uint8_t pointerPos;
    int8_t min;
    int8_t max;
};

subMenu subMenuM;

thermistor ntc(NTC_PIN, 0);


#ifdef SENSOR_SHT31
SHT31 sht;  
#endif

#ifdef SENSOR_BME280
GyverBME280 bme;
#endif


#if SCREEN == 0
#define ERROR
#elif SCREEN == 1
U8G2_SH1106_128X64_NONAME_1_HW_I2C oled(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
#elif SCREEN == 2
U8G2_SSD1306_128X64_NONAME_1_HW_I2C oled(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
#endif

volatile bool prevStatePC3 = false;

#define ENCODER_S1 (A1)
#define ENCODER_S2 (A2)
#define ENCODER_KEY (A3)
EncButton enc(ENCODER_S1, ENCODER_S2, ENCODER_KEY, INPUT, INPUT_PULLUP, LOW);

#if SCALES_MODULE_NUM > 0 && AUTOPID_RUN == 0
#define DT_PIN 8
#define SCK_PIN 9
#define A_PIN 10
#define B_PIN 12
uint8_t sensorNum = 0;
HX711Multi hx711Multi(SCALES_MODULE_NUM, DT_PIN, SCK_PIN, A_PIN, B_PIN);
#elif SCALES_MODULE_NUM != 0 && AUTOPID_RUN == 1
uint32_t zero_weight_eep[] EEMEM{
    0,
    0,
    0,
    0,
};
uint32_t offset_eep[] EEMEM{
    0,
    0,
    0,
    0,
};
uint32_t scale_temp_offset_table_eep[4][6] EEMEM = {
    { 60, 70, 80, 90, 100, 110 },
    { 60, 70, 80, 90, 100, 110 },
    { 60, 70, 80, 90, 100, 110 },
    { 60, 70, 80, 90, 100, 110 },
  };
#endif

#if SCALES_MODULE_NUM != 0
volatile uint16_t prevSpoolMass[4] EEMEM{
    0,
    0,
    0,
    0,
};

enum filamentExpense
{
    ZERO,
    DATA_RESET,
    UPDATE_DATA,
    START,
    WAIT_ALERT_MASS_1,
    WAIT_ALERT_MASS_2,
    WAIT_RESET,
};
filamentExpense filamentExpenseFlag[SCALES_MODULE_NUM] = {UPDATE_DATA};

#endif

float Setpoint, Input, Output;
PID pid(&Input, &Output, &Setpoint, 2, 1, 5, DIRECT);

struct Data
{
    unsigned long timestamp = 0;
    float ntcTemp = 0;
    float airTemp = 0;
    float airTempCorrected = 0;
    float airHumidity = 0;
    bool optimalConditionsReachedFlag = false;
    unsigned long startTime = 0;
    unsigned long errorTime28 = 0;
    uint8_t setTemp = 0;
    uint8_t setHumidity = 0;
    uint16_t setTime = 0;
    bool flag = false;
    bool flagScreenUpdate = false;
    bool flagTimeCounter = false;
    uint8_t setFan = 0;
    float Kp = 0.0f;
    float Ki = 0.0f;
    float Kd = 0.0f;
    uint16_t sampleTime = 0;
    uint8_t deltaT = 0;

    bool operator!=(const Data &other) const
    {
        return timestamp != other.timestamp || int(ntcTemp) != int(other.ntcTemp) || int(airTemp) != int(other.airTemp) || int(airHumidity) != int(other.airHumidity);
    }
};

/* 01 */ void heaterOFF();
/* 02 */ void heater(uint16_t Output, uint16_t &dimmer);
/* 03 */ void heaterON();
void fanMAX();
void fanOFF();
void fanON(int percent);
/* 04 */ void updateIDyerData();
/* 05 */
/* 06 */ void screen(struct subMenu *subMenu);
/* 07 */ void controlsHandler(const menuS constMenu[], uint16_t editableMenu[], const ptrFunc functionMenu[], struct subMenu *subMenu);
/* 08 */ void submenuHandler(const menuS constMenu[], uint8_t menuSize, struct subMenu *subMenu);
/* 09 */ void piii(uint16_t time_ms);
void async_piii(uint16_t time_ms);
/* 10 */ void dryStart();
/* 11 */ void storageStart();
/* 12 */ void autoPidM();
/* 13 */
/* 14 */ uint32_t readError();
/* 15 */ void setError(uint8_t errorCode);
/* 16 */ void displayPrint(struct subMenu *subMenu);
/* 17 */ void displayPrintMode();
void pwm_test();
void offset_set_by_num(uint8_t numSensor);
void zero_set_by_num(uint8_t numSensor);
void setSpool1();
void setSpool2();
void setSpool3();
void setSpool4();
void filamentCheck(uint8_t sensorNum, int16_t mass, stateS state, volatile uint16_t spoolMassArray[]);
/* 18 */ // PID TUNING;
/* 20 */ void scaleShow();
// /* 21 */ void getdataAndSetpoint();
void getData();
void setPoint();
void screenUpdate();
void drawLine(const char *text, int lineIndex, bool background = false, bool center = true, int x = 0, int backgroundX = 0);
/* 22 */ // CASE MENU
/* 23 */ // CASE DRY
/* 24 */ // CASE STORAGE
/* 25 */ void autoPid();
/* 26 */ // NTC MIN
/* 27 */ // NTC MAX
/* 28 */ // BME MIN
/* 29 */ // BME MAX
/* 30 */ // ADC ERROR
/* 0 */  // ADC ACCUMULATED ERROR
/* 31 */ // iDryer.getData
void WDT(uint16_t time, uint8_t current_function_uuid);
void WDT_DISABLE();
void calibration();

class iDryer
{
public:
    Data data;
    Data oldData;
    bool getData()
    {
        data.timestamp = millis();
        data.ntcTemp = (ntc.analog2temp() + data.ntcTemp) / 2.0f;
        
        #ifdef SENSOR_SHT31
        if (sht.dataReady())
        {
            sht.read();
            data.airTemp = (sht.getTemperature() + data.airTemp) / 2.0f;
            data.airHumidity = (sht.getHumidity() + data.airHumidity) / 2.0f;
        }
        #endif

        #ifdef SENSOR_BME280
        data.airTemp = (bme.readTemperature() + data.airTemp) / 2.0f;
        data.airHumidity = (bme.readHumidity() + data.airHumidity) / 2.0f;
        #endif


        if (data.airTemp <= MIN_CALIB_TEMP)
        {
            data.airTempCorrected = data.airTemp;
        }
        else
        {
            data.airTempCorrected = math::map_to_range(data.airTemp, MIN_CALIB_TEMP, MAX_CALIB_TEMP, REAL_CALIB_TEMP_MIN, REAL_CALIB_TEMP_MAX);
        }

        if (data != oldData && data.timestamp - screenTime > SCREEN_UPADATE_TIME)
        {
            screenTime = data.timestamp;
            data.flagScreenUpdate = true;
            oldData = data;
        }
        else
        {
            data.flagScreenUpdate = false;
        }

        if (uint8_t(ceil(data.airTempCorrected)) >= data.setTemp && !data.flagTimeCounter)
            data.flagTimeCounter = true;

        if (data.ntcTemp < TMP_MIN)
            return false;

        if (data.ntcTemp > TMP_MAX + TMP_SAFETY_THRESHOLD)
            return false;

        if (data.airTempCorrected < TMP_MIN)
            return false;

        if (data.airTempCorrected > TMP_MAX + TMP_SAFETY_THRESHOLD)
            return false;

        return true;
    }

} iDryer;

servo Servo(SERVO_1_PIN, eeprom_read_word(&menuVal[DEF_SERVO_CLOSED]), eeprom_read_word(&menuVal[DEF_SERVO_OPEN]), eeprom_read_word(&menuVal[DEF_SERVO_CORNER]));

BuzzerController buzzer(BUZZER_PIN);

void servoTest()
{
    updateIDyerData();
    Servo.toggle();
}

#ifdef v220V
void isr()
{
#ifdef DEBUG
    testTIMER_COUNT++;
#endif
    PORTD &= ~(1 << DIMMER_PIN);
    if ((state == DRY || state == STORAGE || state == AUTOPID) && Servo.state != MOVE && dimmer >= HEATER_MIN && dimmer < HEATER_MAX)
    {
        // if (lastDim != dimmer)
        // {
        //     Timer1.setPeriod(lastDim = dimmer);
        // }
        // else
        // {
        //     Timer1.restart();
        // }
        Timer1.setPeriod(dimmer);
        timer1_dimmerFlag = true;
    }
    else
    {
        timer1_dimmerFlag = false;
    }
}

ISR(TIMER1_A)
{
    if (Servo.state != MOVE && timer1_dimmerFlag)
    {
        PORTD |= (1 << DIMMER_PIN);
        timer1_dimmerFlag = false;
        Timer1.stop();
        // if (timer1_dimmerFlag)
        // {
        //     if (dimmer)
        //         // digitalWrite(DIMMER_PIN, 1);
        //         PORTD |= (1 << DIMMER_PIN);
        //     timer1_dimmerFlag = 0;
        //     // Timer1.setPeriod(50);
        //     Timer1.stop();
        // }
        // else
        // {
        //     // digitalWrite(DIMMER_PIN, 0);
        //     // PORTD &= ~(1 << DIMMER_PIN);
        // }
    }
    else if (Servo.state == MOVE)
    {
        Servo.updateServo();
    }
}
#endif

ISR(PCINT1_vect)
{
    if (state == MENU)
        scaleTimer = millis();

    uint8_t pinStateC = PINC;

    if (pinStateC & (1 << PC1) || pinStateC & (1 << PC2))
    {
        enc.tickISR();
    }

    bool currentStatePC3 = PINC & (1 << PC3);

    if (currentStatePC3 != prevStatePC3)
    {
        enc.readBtn();
        prevStatePC3 = currentStatePC3;
    }
}

char *printMenuItem(PGM_P const *text)
{
    static char buffer[22];
    uint16_t ptr = pgm_read_word(text);
    uint8_t i = 0;
    do
    {
        buffer[i] = (char)(pgm_read_byte(ptr++));
    } while (i++ != 22);
    return buffer;
}

//!! Передать menuPGM аргументом
void displayPrint(struct subMenu *subMenu)
{
    WDT(WDTO_1S, 16);
#ifdef DEBUG
    WDT_DISABLE();
#endif
    oled.firstPage();
    do
    {
        auto isTopLevelItem = subMenuM.parentID == 0;

        uint8_t maxPos = SCREEN_LINES - MENU_HEADER < subMenu->membersQuantity ? SCREEN_LINES - MENU_HEADER : subMenu->membersQuantity;

        drawLine(printMenuItem(&menuTxt[subMenuM.parentID]), 1, false, true, isTopLevelItem ? -10 : 0);

        if (isTopLevelItem)
        {
            char val[6];
            snprintf(val, sizeof(val), "%2hu/%2hu", (uint16_t)iDryer.data.airHumidity, (uint16_t)iDryer.data.airTempCorrected);
#ifdef DEBUG
            snprintf(val, sizeof(val), "%4d", oldTime1 - oldTime2);
#endif
            drawLine(val, 1, false, false, 88);
            snprintf(val, sizeof(val), "%2hu", (uint16_t)iDryer.data.ntcTemp);
#ifdef DEBUG
            snprintf(val, sizeof(val), "%3hu", dimmer);
#endif
            drawLine(val, 1, false, false);
        }
        oled.drawLine(0, LINE_HIGHT + 2, 128, LINE_HIGHT + 2);
        for (uint8_t i = 0; i < maxPos; i++)
        {
            drawLine(printMenuItem(&menuTxt[subMenuM.membersID[subMenuM.linesToScreen[i]]]), i + 2, false, false);

            char val[6] = {};
            if (pgm_read_word(&menuPGM[subMenuM.membersID[subMenuM.linesToScreen[i]]].min) ||
                pgm_read_word(&menuPGM[subMenuM.membersID[subMenuM.linesToScreen[i]]].max) ||
                eeprom_read_word(&menuVal[subMenuM.membersID[subMenuM.linesToScreen[i]]]))
            {
                snprintf(val, sizeof(val), "%5u", eeprom_read_word(&menuVal[subMenuM.membersID[subMenuM.linesToScreen[i]]]));
            }

            drawLine(val, i + 2, subMenuM.pointerPos == i, false, 88, subMenu->changeVal ? 96 : 0);
        }
    } while (oled.nextPage());
    iDryer.data.flagScreenUpdate = false;
    WDT_DISABLE();
}

void displayPrintMode()
{
    WDT(WDTO_4S, 17);

    oled.firstPage();
    do
    {
        uint8_t text = 0;
        if (state == DRY)
        {
            text = DEF_MENU_DRYING;
        }
        if (state == STORAGE)
        {
            text = DEF_MENU_STORAGE;
        }

        char val[8];
        drawLine(printMenuItem(&menuTxt[text]), 1);

        snprintf(val, sizeof(val), "%2hu", iDryer.data.setTemp);
        drawLine(val, 1, false, false);
        snprintf(val, sizeof(val), "%3hu", text == DEF_MENU_DRYING ? iDryer.data.setTime : iDryer.data.setHumidity);
        drawLine(val, 1, true, false, 104);

        drawLine(printMenuItem(&serviceTxt[6]), 2, false, false, 0);
        snprintf(val, sizeof(val), "%3hu/%03hu", uint8_t(iDryer.data.airTempCorrected), uint8_t(iDryer.data.airTemp));
        drawLine(val, 2, false, false, 72);

        drawLine(printMenuItem(&serviceTxt[7]), 3, false, false, 0);
        snprintf(val, sizeof(val), "%3hu/%03hu", uint8_t(iDryer.data.ntcTemp), uint8_t(Setpoint));
        drawLine(val, 3, false, false, 72);

        drawLine(printMenuItem(&serviceTxt[8]), 4, false, false, 0);
        snprintf(val, sizeof(val), "%3hu", uint8_t(iDryer.data.airHumidity));
        drawLine(val, 4, false, false, 104);
    } while (oled.nextPage());
    iDryer.data.flagScreenUpdate = false;
    WDT_DISABLE();
}

void setup()
{
    WDT_DISABLE();

#ifdef KASYAK_FINDER
    Serial.begin(9600);
#endif

    pinMode(SERVO_1_PIN, OUTPUT);
    PORTD &= ~(1 << SERVO_1_PIN);
    // digitalWrite(SERVO_1_PIN, 0);

    DDRC &= ~((1 << PC1) | (1 << PC2) | (1 << PC3));
    PORTC |= (1 << PC1) | (1 << PC2) | (1 << PC3);

    PCICR |= (1 << PCIE1);
    PCMSK1 |= (1 << PCINT9) | (1 << PCINT10) | (1 << PCINT11);

    sei();

#ifdef v220V
    dimmer = HEATER_MAX;

    pinMode(ZERO_PIN, INPUT_PULLUP);
    pinMode(DIMMER_PIN, OUTPUT);
#else
    pinMode(DIMMER_PIN, OUTPUT);
    digitalWrite(DIMMER_PIN, 0);
#endif

    TCCR2B = TCCR2B_PRESCALER;
    TCCR2A = TCCR2A_MODE;

    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, 0);
    pinMode(FAN, OUTPUT);
    digitalWrite(FAN, 0);
#if SCALES_MODULE_NUM != 0
    pinMode(FILAMENT_SENSOR, OUTPUT);
    digitalWrite(FILAMENT_SENSOR, 0);
#endif

    oled.begin();
    oled.setFlipMode(SCREEN_FLIP);
    oled.enableUTF8Print();
    oled.setFont(u8g2_font);

    oled.firstPage();
    do
    {
        drawLine(printMenuItem(&serviceTxt[DEF_T_AYRA]), 1);
        drawLine(printMenuItem(&serviceTxt[DEF_T_VERSION]), 2);
        drawLine(printMenuItem(&serviceTxt[DEF_T_VER]), 3);
    } while (oled.nextPage());

    delay(3000);

    fanOFF();

    menuSize = sizeof(menuTxt) / sizeof(menuTxt[0]);

    uint16_t test = eeprom_read_word(&menuVal[0]);

    if (test != EEPROM_VALIDATION_VALUE)
    {
        setError(31);
    }

    uint32_t errorCode = readError();

    if (errorCode)
    {
        fanON(100);

        oled.clear();

        do
        {
            drawLine(printMenuItem(&serviceTxt[DEF_T_ERROR]), 1, true);

            uint8_t charCounter = 0;
            for (uint8_t i = 0; i < sizeof(errorCode) * 8; i++)
            {
                if (errorCode & (1U << i))
                {
                    snprintf(serviceString, sizeof(serviceString), "%2d", i);
                    drawLine(serviceString, charCounter / 4 + 2, true, false, (charCounter * 32) % 128 + 4);
                    if (charCounter % 5 == 0)
                        snprintf(serviceString, sizeof(serviceString), "%s", "");
                    charCounter++;
                }
            }
        } while (oled.nextPage());

        while (digitalRead(encBut))
        {
            piii(500);
        }

        WDT(WDTO_15MS, 0);
        while (true)
        {
        }
    }

#if OVERWRITE_PID == 1
    eeprom_update_word(&menuVal[DEF_PID_KP], K_PROPRTIONAL);
    eeprom_update_word(&menuVal[DEF_PID_KI], K_INTEGRAL);
    eeprom_update_word(&menuVal[DEF_PID_KD], K_DERIVATIVE);
    eeprom_update_word(&menuVal[DEF_AVTOPID_TIME_MS], K_SAMPLE_TIME);
#endif

    updateIDyerData();

    enc.setEncISR(true);
    enc.setEncType(MY_ENCODER_TYPE);
    // enc.setFastTimeout(30);
    enc.setEncReverse(ENCODER_REVERSE);
    enc.counter = 0;

    // bme.setFilter(FILTER_COEF_16);
    // bme.setStandbyTime(STANDBY_250MS);

    #ifdef SENSOR_SHT31
    while (!sht.begin())
    {
        piii(300);
    }
    #endif
    
    #ifdef SENSOR_BME280
    while (!bme.begin(0x76))
    {
        piii(300);
    }
    #endif

    while (analogRead(NTC_PIN) < ADC_MIN || analogRead(NTC_PIN) > ADC_MAX)
    {
        piii(1000);
    }

    // Для первоначального обновления меню
    subMenuM.levelUpdate = DOWN;
    subMenuM.pointerUpdate = 1;
    subMenuM.parentID = 0;
    subMenuM.position = 0;
    memset(subMenuM.membersID, 0, sizeof(subMenuM.membersID) / sizeof(subMenuM.membersID[0]));

    uint8_t i = 10;
    while (i)
    {
        iDryer.getData();
        i--;
    }

    Servo.close();

#ifdef PWM_TEST
    pwm_test();
#endif
#if AUTOPID_RUN == 1
    // Запись массивов в EEPROM
    eeprom_write_dword(&zero_weight_eep[0], 1);
    eeprom_write_dword(&zero_weight_eep[1], 1);
    eeprom_write_dword(&offset_eep[0], 1);
    eeprom_write_dword(&offset_eep[1], 1);

    autoPidM();
    autoPid();

    oled.firstPage();
    do
    {
        drawLine(printMenuItem(&serviceTxt[DEF_T_BURN]), 2);
        drawLine(printMenuItem(&serviceTxt[DEF_T_PART]), 3);
    } while (oled.nextPage());
    delay(300000);
#endif
#if SCALES_MODULE_NUM != 0 && AUTOPID_RUN == 0
    hx711Multi.begin(128);
#endif
}

void loop()
{

    // calibration();
    enc.tick();
    buzzer.update();

    tmpTemp = (tmpTemp * 9 + analogRead(NTC_PIN)) / 10;
    if (tmpTemp <= ADC_MIN || tmpTemp >= ADC_MAX)
    {
        if (state == DRY || state == STORAGE)
        {
            setError(30);
            state = NTC_ERROR;
        }
    }

#if SCALES_MODULE_NUM != 0 && AUTOPID_RUN == 0

    hx711Multi.readMassMulti();
    filamentCheck(sensorNum, hx711Multi.getMassMulti(sensorNum), state, prevSpoolMass);
    sensorNum++;

    if (sensorNum >= SCALES_MODULE_NUM)
        sensorNum = 0;
#endif

    if (enc.hold())
    {
        if (state == DRY || state == STORAGE)
        {
            WDT_DISABLE();
            state = MENU;
            heaterOFF();
            async_piii(500);
            subMenuM.levelUpdate = UP;
            subMenuM.parentID = 0;
            subMenuM.pointerPos = 0;
            subMenuM.pointerUpdate = 1;
            while (digitalRead(encBut))
                ;
        }
    }

    switch (state)
    {
    case NTC_ERROR:

        fanMAX();
        heaterOFF();

        oled.clear();
        oled.firstPage();
        do
        {
            drawLine(printMenuItem(&serviceTxt[DEF_T_ERROR]), 2, true);
            drawLine(printMenuItem(&serviceTxt[DEF_T_CHECK]), 2, true);
            drawLine(printMenuItem(&serviceTxt[DEF_T_THERMISTOR]), 2, true);
            drawLine(printMenuItem(&serviceTxt[DEF_T_OVERLOAD]), 2, true);
        } while (oled.nextPage());

        while (digitalRead(encBut))
        {
            async_piii(500);
            delay(500);
        }
        break;
    case OFF:

        break;
    case ON:

        break;
    case MENU:
        WDT(WDTO_8S, 22);
        if (iDryer.getData())
        {
            if (iDryer.data.ntcTemp > 45)
                fanMAX();
            else if (iDryer.data.ntcTemp < 40)
                fanOFF();
        }

#if SCALES_MODULE_NUM != 0 && AUTOPID_RUN == 0
        if (hx711Multi.tare[hx711Multi.sensorNum] != (uint8_t)eeprom_read_word(&menuVal[DEF_COIL_1_TARA + (hx711Multi.sensorNum * 3)]))
        {
            hx711Multi.tare[hx711Multi.sensorNum] = (uint8_t)eeprom_read_word(&menuVal[DEF_COIL_1_TARA + (hx711Multi.sensorNum * 3)]);
        }
#endif

        controlsHandler(menuPGM, menuVal, menuFunc, &subMenuM);

#if SCALES_MODULE_NUM != 0 && AUTOPID_RUN == 0

        if (millis() - scaleTimer > MENU_SCALE_SWITCH_TIME && subMenuM.parentID == 0)
        {
            scaleShow();
        }
        else
        {
#endif
            if (subMenuM.levelUpdate)
            {
                submenuHandler(menuPGM, menuSize, &subMenuM);
                screen(&subMenuM);
                displayPrint(&subMenuM);
            }

            if (subMenuM.pointerUpdate || iDryer.data.flagScreenUpdate) // TODO: проверить
            {
                screen(&subMenuM);
                displayPrint(&subMenuM);
            }

#if SCALES_MODULE_NUM != 0 && AUTOPID_RUN == 0
        }
#endif
        WDT_DISABLE();
        break;

    case DRY:
        WDT(WDTO_1S, 23);
        getData();
        setPoint();
        screenUpdate();
        fanON(iDryer.data.setFan);
        Input = iDryer.data.ntcTemp;
        pid.Compute();
        heater(Output, dimmer);

        if (Setpoint == 0)
        {
            dimmer = HEATER_OFF;
        }

        if (iDryer.data.timestamp - oldTimer >= 60000 && iDryer.data.flagTimeCounter)
        {
            // DEBUG_PRINT(4);
            oldTimer = iDryer.data.timestamp;
            iDryer.data.setTime--;
        }

        if (iDryer.data.setTime == 0)
        {
            async_piii(1000);
            // DEBUG_PRINT(5);
            subMenuM.parentID = 5; // ID пункта хранение
            heaterOFF();
            WDT_DISABLE();
            storageStart();
            // DEBUG_PRINT(6);
        }

        Servo.check();
        // DEBUG_PRINT(7);
        WDT_DISABLE();

        break;
#ifndef KASYAK_FINDER
    case STORAGE:

        WDT(WDTO_4S, 24);
        getData();
        setPoint();
        screenUpdate();

        if (iDryer.data.flag)
        {
            iDryer.data.flagTimeCounter ? fanON(iDryer.data.setFan) : fanMAX();
            Input = iDryer.data.ntcTemp;
            pid.Compute();
            heater(Output, dimmer);
            Servo.check();

            if (iDryer.data.setTemp <= iDryer.data.airTempCorrected && iDryer.data.airHumidity <= iDryer.data.setHumidity)
            {
                if (Servo.state == OPEN)
                    Servo.toggle();

                iDryer.data.optimalConditionsReachedFlag = true;
                iDryer.data.flag = false;
            }
        }
        else
        {
            if (Servo.state == CLOSED && iDryer.data.optimalConditionsReachedFlag == true)
            {
                iDryer.data.optimalConditionsReachedFlag = false;
                heaterOFF();
            }

            if (iDryer.data.ntcTemp <= iDryer.data.airTempCorrected + TEMP_HYSTERESIS)
            {
                fanOFF();
            }
#if ACTIVATION_HYSTERESIS_MODE == 1
            if ((iDryer.data.airHumidity >= iDryer.data.setHumidity + HUMIDITY_HYSTERESIS && !iDryer.data.flag) || (iDryer.data.airTemp <= iDryer.data.setTemp - TEMP_HYSTERESIS && !iDryer.data.flag))
#else
            if (iDryer.data.airHumidity >= iDryer.data.setHumidity + HUMIDITY_HYSTERESIS && !iDryer.data.flag)
#endif
            {
                iDryer.data.flag = true;
                fanON(iDryer.data.setFan);
                heaterON();
            }
        }

        WDT_DISABLE();
        break;
#endif

#if SCALES_MODULE_NUM == 0
    case AUTOPID:
        autoPidM();
#ifndef PWM_TEST
        autoPid();
#endif
#endif
        break;

    default:
        break;
    }
}

void controlsHandler(const menuS constMenu[], uint16_t editableMenu[], const ptrFunc functionMenu[], struct subMenu *subMenu)
{
    WDT(WDTO_250MS, 7);
    if (enc.click())
    {
        subMenuM.pointerUpdate = 1;

        if (!pgm_read_word(&constMenu[subMenu->membersID[subMenu->position]].min) &&
            !pgm_read_word(&constMenu[subMenu->membersID[subMenu->position]].max) &&
            !functionMenu[subMenu->membersID[subMenu->position]]) // меню
        {
            subMenu->level + 1 > subMenu->levelMax ? subMenu->level : subMenu->level++;
            subMenu->levelUpdate = DOWN;
        }
        else
        {
            if (functionMenu[subMenu->membersID[subMenu->position]]) // подменю с функцией
            {
                functionMenu[subMenu->membersID[subMenu->position]]();
            }
            else // подменю с изменяемыми параметрами
            {
                subMenu->changeVal = !subMenu->changeVal;
            }
        }
    }

    if (enc.hold())
    {
        switch (subMenu->changeVal)
        {
        case false:
            subMenu->level == subMenu->min ? subMenu->level : subMenu->level--;
            subMenu->levelUpdate = UP;
            break;
        case true:
            subMenu->changeVal = !subMenu->changeVal;
            break;

        default:
            break;
        }
    }

    if (enc.right())
    {
        uint8_t inc = 1;
        if (enc.fast())
            inc *= ENCODER_FAST_INCREMENT;

        if (!subMenu->changeVal)
        {
            subMenu->position == subMenu->max ? subMenu->position = 0 : subMenu->position++;
        }
        else
        {
            uint16_t val = eeprom_read_word(&editableMenu[subMenuM.membersID[subMenuM.linesToScreen[subMenuM.pointerPos]]]);
            if (val + inc <=
                pgm_read_word(&constMenu[subMenuM.membersID[subMenuM.linesToScreen[subMenuM.pointerPos]]].max))
                val += inc;
            if (val > pgm_read_word(&constMenu[subMenuM.membersID[subMenuM.linesToScreen[subMenuM.pointerPos]]].max))
                val = pgm_read_word(&constMenu[subMenuM.membersID[subMenuM.linesToScreen[subMenuM.pointerPos]]].max);

            eeprom_write_word((uint16_t *)&editableMenu[subMenuM.membersID[subMenuM.linesToScreen[subMenuM.pointerPos]]], val);
        }
        subMenu->pointerUpdate = true;
    }

    if (enc.left())
    {
        int8_t inc = 1;
        if (enc.fast())
            inc *= ENCODER_FAST_INCREMENT;

        if (!subMenu->changeVal)
        {
            subMenu->position == subMenu->min ? subMenu->position = subMenu->max : subMenu->position--;
        }
        else
        {
            uint16_t val = eeprom_read_word(&editableMenu[subMenuM.membersID[subMenuM.linesToScreen[subMenuM.pointerPos]]]);
            if ((int16_t)val - (int16_t)inc >=
                (int16_t)pgm_read_word(&constMenu[subMenuM.membersID[subMenuM.linesToScreen[subMenuM.pointerPos]]].min))
                val -= inc;

            eeprom_write_word((uint16_t *)&editableMenu[subMenuM.membersID[subMenuM.linesToScreen[subMenuM.pointerPos]]], val);
        }
        subMenu->pointerUpdate = true;
    }
    WDT_DISABLE();
}

void submenuHandler(const menuS constMenu[], uint8_t menuSize, struct subMenu *subMenu)
{
    WDT(WDTO_250MS, 8);
    uint8_t posTmp = subMenu->parentID;

    if (subMenu->levelUpdate == DOWN)
    {
        subMenu->parentID = subMenu->membersID[subMenu->position];
        subMenu->position = 0;
        subMenu->pointerPos = 0;
    }
    if (subMenu->levelUpdate == UP)
    {
        subMenu->parentID = pgm_read_byte(&constMenu[subMenu->parentID].parentID);
    }

    subMenu->levelUpdate = NOTHING;

    memset(subMenuM.membersID, 0, sizeof(subMenuM.membersID) / sizeof(subMenuM.membersID[0]));
    int8_t iCounter = 0;

    for (uint8_t i = 0; i < menuSize; i++)
    {
        if (pgm_read_byte(&constMenu[i].parentID) == subMenu->parentID && pgm_read_byte(&constMenu[i].id) != 0)
        {
            subMenu->membersID[iCounter] = i;
            if (i == posTmp)
            {
                subMenu->position = iCounter;
                subMenu->pointerUpdate = 1;
            }
            iCounter++;
        }
    }
    subMenu->membersQuantity = iCounter;
    subMenu->min = 0;
    iCounter > 0 ? subMenu->max = iCounter - 1 : 0;
    WDT_DISABLE();
}

void screen(struct subMenu *subMenu)
{
    WDT(WDTO_500MS, 6);
    uint8_t maxPos = SCREEN_LINES - MENU_HEADER <= subMenu->membersQuantity ? SCREEN_LINES - MENU_HEADER - 1 : subMenu->membersQuantity - 1;
    uint8_t pointerPos = constrain(subMenu->position, 0, maxPos);
    uint8_t screenStartPos = subMenu->position < SCREEN_LINES - MENU_HEADER - 1 ? 0 : subMenu->position - maxPos;

    memset(subMenuM.linesToScreen, 0, sizeof(subMenuM.linesToScreen) / sizeof(subMenuM.linesToScreen[0]));

    for (uint8_t i = screenStartPos; i <= screenStartPos + maxPos; i++)
    {
        if (i - screenStartPos == pointerPos)
        {
            subMenu->pointerPos = i - screenStartPos;
        }
        subMenu->linesToScreen[i - screenStartPos] = i;
    }

    subMenu->pointerUpdate = 0;
    WDT_DISABLE();
}

void mainDryStart()
{
    iDryer.data.setTemp = eeprom_read_word(&menuVal[DEF_DRYING_TEMPERATURE]);
    iDryer.data.setTime = eeprom_read_word(&menuVal[DEF_DRYING_TIME]);
    dryStart();
}

void plaDryStart()
{
    iDryer.data.setTemp = eeprom_read_word(&menuVal[DEF_PLA_TEMPERATURE]);
    iDryer.data.setTime = eeprom_read_word(&menuVal[DEF_PLA_TIME]);
    dryStart();
}

void petgDryStart()
{
    iDryer.data.setTemp = eeprom_read_word(&menuVal[DEF_PETG_TEMPERATURE]);
    iDryer.data.setTime = eeprom_read_word(&menuVal[DEF_PETG_TIME]);
    dryStart();
}

void absDryStart()
{
    iDryer.data.setTemp = eeprom_read_word(&menuVal[DEF_ABS_TEMPERATURE]);
    iDryer.data.setTime = eeprom_read_word(&menuVal[DEF_ABS_TIME]);
    dryStart();
}

void dryStart()
{
    WDT(WDTO_4S, 10);
    oldTimer = 0;
    scaleTimer = millis();
    heaterON();

    state = DRY;

    iDryer.data.flag = true;
    iDryer.data.flagTimeCounter = false;
    iDryer.data.flagScreenUpdate = true;

    iDryer.data.startTime = millis();

    WDT_DISABLE();
}

void storageStart()
{
    WDT(WDTO_4S, 11);
    oldTimer = 0;
    scaleTimer = millis();
    heaterON();

    state = STORAGE;

    iDryer.data.flag = true;
    iDryer.data.flagTimeCounter = false;
    iDryer.data.flagScreenUpdate = true;
    iDryer.data.setTemp = eeprom_read_word(&menuVal[DEF_STORAGE_TEMPERATURE]);
    iDryer.data.setHumidity = eeprom_read_word(&menuVal[DEF_STORAGE_HUMIDITY]);

    WDT_DISABLE();
}

void autoPidM()
{
}

void updateIDyerData()
{
    WDT(WDTO_250MS, 4);
    iDryer.data.Kp = (float)eeprom_read_word(&menuVal[DEF_PID_KP]) / 100.0f;
    iDryer.data.Ki = (float)eeprom_read_word(&menuVal[DEF_PID_KI]) / 100.0f;
    iDryer.data.Kd = (float)eeprom_read_word(&menuVal[DEF_PID_KD]) / 100.0f;
    iDryer.data.sampleTime = eeprom_read_word(&menuVal[DEF_AVTOPID_TIME_MS]);
    iDryer.data.deltaT = eeprom_read_word(&menuVal[DEF_SETTINGS_DELTA]);
    iDryer.data.setHumidity = eeprom_read_word(&menuVal[DEF_STORAGE_HUMIDITY]);
    iDryer.data.setFan = eeprom_read_word(&menuVal[DEF_SETTINGS_BLOWING]);

    pid.SetMode(AUTOMATIC);             // MANUAL AUTOMATIC
    pid.SetControllerDirection(DIRECT); // REVERSE
    pid.SetOutputLimits((float)HEATER_MIN, (float)HEATER_MAX);
    pid.SetTunings(iDryer.data.Kp, iDryer.data.Ki, iDryer.data.Kd, PID_TYPE);
    pid.SetSampleTime((int)iDryer.data.sampleTime);

    Servo.set(eeprom_read_word(&menuVal[DEF_SERVO_CLOSED]), eeprom_read_word(&menuVal[DEF_SERVO_OPEN]), eeprom_read_word(&menuVal[DEF_SERVO_CORNER]));
    // Servo.toggle();
    WDT_DISABLE();
}


void saveAll()
{
    detachInterrupt(INT_NUM);
    timer1_dimmerFlag = false;
    dimmer = HEATER_OFF;
    digitalWrite(DIMMER_PIN, 0);

    updateIDyerData();

    oled.firstPage();
    do
    {
        drawLine(printMenuItem(&serviceTxt[DEF_T_AYRA]), 3);
    } while (oled.nextPage());
    delay(500);
    subMenuM.pointerUpdate = 1;
}

// WDTO_15MS
// WDTO_8S
void WDT(uint16_t time, uint8_t current_function_uuid)
{
    globalErrorFuncUUID = current_function_uuid;
    wdt_reset();
    wdt_disable();
    wdt_enable(time);
    WDTCSR |= (1 << WDIE);
}

ISR(WDT_vect)
{
    setError(globalErrorFuncUUID);
}

void WDT_DISABLE()
{
    MCUSR = 0;
    wdt_reset();
    wdt_disable();
}

void setError(uint8_t errorCode)
{
    auto wdtError = readError();

    if (errorCode != 0)
    {
        wdtError |= (1U << errorCode);
    }
    else
    {
        wdtError = 0;
    }

    eeprom_write_dword(&ERROR_CODE, wdtError);
}

uint32_t readError()
{
    return eeprom_read_dword(&ERROR_CODE);
}

void piii(uint16_t time_ms)
{
    if (state == AUTOPID)
    {
        return;
    }

    time_ms = max(10u, time_ms);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(time_ms);
    digitalWrite(BUZZER_PIN, LOW);
    delay(time_ms);
}

void async_piii(uint16_t time_ms)
{
    if (state == AUTOPID)
    {
        return;
    }

    buzzer.buzz(time_ms);
}

void heater(uint16_t Output, uint16_t &dimmer)
{
    WDT(WDTO_250MS, 2);
#ifdef v220V
    dimmer = uint16_t(HEATER_MAX + HEATER_MIN - Output);
#else
    analogWrite(DIMMER_PIN, Output);
#endif
    WDT_DISABLE();
}

void heaterON()
{
    WDT(WDTO_250MS, 3);
#ifdef v220V
    attachInterrupt(INT_NUM, isr, RISING);
    dimmer = HEATER_MAX;
#else
    dimmer = HEATER_MIN;
    digitalWrite(DIMMER_PIN, HEATER_MIN);
#endif

#ifdef HEATER_ON_SOUND_NOTIFICATION
    piii(100);
#endif
    WDT_DISABLE();
}

void fanMAX()
{
    fanON(100);
}

void fanOFF()
{
    fanON(0);
}

void fanON(int percent)
{
    analogWrite(FAN, map(percent, 0, 100, 0, 255));
}

void heaterOFF()
{
    WDT(WDTO_250MS, 1);
#ifdef v220V
    detachInterrupt(INT_NUM);
    timer1_dimmerFlag = false;
    dimmer = HEATER_OFF;
    digitalWrite(DIMMER_PIN, 0);
#else
    digitalWrite(DIMMER_PIN, 0);
#endif
    WDT_DISABLE();
}

void pwm_test()
{
    uint16_t frequency = 0;
    for (uint8_t i = 0; i < 11; i++)
    {
        switch (i)
        {
        case 0: // 30
            frequency = 30;
            TCCR2B = 0b00000111;
            TCCR2A = 0b00000001;
            break;
        case 1: // 60
            frequency = 60;
            TCCR2B = 0b00000111;
            TCCR2A = 0b00000011;
            break;
        case 2: // 122
            frequency = 122;
            TCCR2B = 0b00000110;
            TCCR2A = 0b00000001;
            break;
        case 3: // 245
            frequency = 245;
            TCCR2B = 0b00000101;
            TCCR2A = 0b00000001;
            break;
        case 4: // 488
            frequency = 488;
            TCCR2B = 0b00000100;
            TCCR2A = 0b00000001;
            break;
        case 5: // 980
            frequency = 980;
            TCCR2B = 0b00000011;
            TCCR2A = 0b00000001;
            break;
        case 6: // 2000
            frequency = 2000;
            TCCR2B = 0b00000011;
            TCCR2A = 0b00000011;
            break;
        case 7: // 4000
            frequency = 4000;
            TCCR2B = 0b00000010;
            TCCR2A = 0b00000001;
            break;
        case 8: // 8000
            frequency = 8000;
            TCCR2B = 0b00000010;
            TCCR2A = 0b00000011;
            break;
        case 9: // 31400
            frequency = 31400;
            TCCR2B = 0b00000001;
            TCCR2A = 0b00000001;
            break;
        case 10: // 65200
            frequency = 65200;
            TCCR2B = 0b00000001;
            TCCR2A = 0b00000011;
            break;

        default:
            break;
        }
        uint8_t k = 100;
        while (k > 0)
        {
            fanOFF();
            delay(2000);
            fanON(k);
            oled.clear();
            oled.firstPage();
            do
            {
                snprintf(serviceString, sizeof(serviceString), "%s %6hu", printMenuItem(&serviceTxt[DEF_T_FREQUENCY]), frequency);
                drawLine(serviceString, 2);
                snprintf(serviceString, sizeof(serviceString), "%4s %6hu", printMenuItem(&menuTxt[DEF_SETTINGS_BLOWING]), k);
                drawLine(serviceString, 3);
            } while (oled.nextPage());
            delay(3000);
            k -= 10;
        }
    }
}

void screenUpdate()
{
#if SCALES_MODULE_NUM == 0 && AUTOPID_RUN == 0
    if (iDryer.data.flagScreenUpdate)
        displayPrintMode();
#endif
#if SCALES_MODULE_NUM != 0 && AUTOPID_RUN == 0
    if (millis() - scaleTimer > MENU_SCALE_SWITCH_TIME)
    {
        scaleTimer = millis();
        isScaleShow = !isScaleShow;

        displayPrintMode();

        // if (isScaleShow)
        // {
        //     scaleShow();
        // }
        // else
        // {
        //     displayPrintMode();
        // }
    }
#endif
}

void drawLine(const char *text, int lineIndex, bool background, bool center, int x, int backgroundX)
{
    auto y = LINE_HIGHT * lineIndex;

    if (center)
    {
        x += (128 - oled.getUTF8Width(text)) / 2;
    }

    oled.drawUTF8(x, y, text);

    if (background)
    {
        oled.drawButtonUTF8(backgroundX, y, U8G2_BTN_INV, 128, 0, PADDING_V, "");
    }
}

void getData()
{
    if (!iDryer.getData())
    {
        ERROR_COUNTER++;
        if (ERROR_COUNTER > MAX_ERROR)
        {
            if (iDryer.data.ntcTemp < TMP_MIN)
                setError(26);

            if (iDryer.data.ntcTemp > TMP_MAX + 10)
                setError(27);

            if (iDryer.data.airTempCorrected < TMP_MIN)
                setError(28);

            if (iDryer.data.airTempCorrected > TMP_MAX + 10)
                setError(29);

            setError(0);
            state = NTC_ERROR;
        }
    }
    else
    {
        ERROR_COUNTER = 0;
    }
}

void setPoint()
{
    auto currentTemp = iDryer.data.airTempCorrected; // Текущая температура
    float desiredTemp = iDryer.data.setTemp;         // Заданная температура
    float deltaT = iDryer.data.deltaT;               // Дополнительный коэффициент для агрессивного нагрева

    auto delta = desiredTemp - currentTemp;
    auto adjustment = math::map_to_range(delta, HEATER_AIR_DELTA, HEATING_THRESHOLD, HEATER_AIR_DELTA, deltaT);

    Setpoint = desiredTemp + adjustment;

    if (Setpoint > TMP_MAX)
    {
        Setpoint = TMP_MAX;
    }

    // Отключение при критическом перегреве
    if (currentTemp >= desiredTemp + CRITICAL_OVERHEAT)
    {
        Setpoint = 0;
    }

#ifdef KASYAK_FINDER
    Serial.print(" t: ");
    Serial.print(iDryer.data.timestamp);
    Serial.print(" d: ");
    Serial.print(delta, 2);
    Serial.print(" a: ");
    Serial.print(adjustment, 2);
    Serial.print(" t: ");
    Serial.print(currentTemp, 2);
    Serial.print(" s: ");
    Serial.print(Setpoint, 2);
    Serial.print(" n: ");
    Serial.print(iDryer.data.ntcTemp, 2);
    Serial.print(" o: ");
    Serial.print(Output, 2);
    Serial.print(" d: ");
    Serial.print(dimmer);
    Serial.println();
    Serial.flush();
#endif
}

void autoPid()
{
}

#if SCALES_MODULE_NUM > 0 && AUTOPID_RUN == 0
void zero_set_by_num(uint8_t numSensor)
{
    hx711Multi.zeroSetupMulti(numSensor);
}

void offset_set_by_num(uint8_t numSensor)
{
    hx711Multi.offsetSetupMulti(numSensor);
}

void setSpool(uint8_t spool)
{
    WDT_DISABLE();

    for (uint8_t i = 0; i < 6; i++)
    {
        oled.firstPage();
        do
        {
            snprintf(serviceString, sizeof(serviceString), "%s %1d", printMenuItem(&serviceTxt[DEF_T_SPOOL]), spool + 1);
            drawLine(serviceString, 2);

            snprintf(serviceString, sizeof(serviceString), "%s", printMenuItem(&serviceTxt[DEF_T_REMOVE]));
            snprintf(serviceString, sizeof(serviceString), "%s %s", serviceString, printMenuItem(&serviceTxt[DEF_T_WEIGHT]));
            drawLine(serviceString, 3);

            snprintf(serviceString, sizeof(serviceString), "%2d ", 5 - i);
            drawLine(serviceString, 4);
        } while (oled.nextPage());
        delay(800);
    }

    oled.firstPage();
    do
    {
        snprintf(serviceString, sizeof(serviceString), "%s %1d", printMenuItem(&serviceTxt[DEF_T_SPOOL]), spool + 1);
        drawLine(serviceString, 2);
        snprintf(serviceString, sizeof(serviceString), "%s", printMenuItem(&serviceTxt[DEF_T_SETUP]));
        drawLine(serviceString, 3);
    } while (oled.nextPage());
    delay(1000);

    zero_set_by_num(spool);

    for (uint8_t i = 0; i < 10; i++)
    {
        oled.firstPage();
        do
        {
            snprintf(serviceString, sizeof(serviceString), "%s %1d", printMenuItem(&serviceTxt[DEF_T_SPOOL]), spool + 1);
            drawLine(serviceString, 1);
            snprintf(serviceString, sizeof(serviceString), "%s", printMenuItem(&serviceTxt[DEF_T_PUT]));
            drawLine(serviceString, 2);
            snprintf(serviceString, sizeof(serviceString), "%s %4d", printMenuItem(&serviceTxt[DEF_T_WEIGHT]), 1000);
            snprintf(serviceString, sizeof(serviceString), "%s %s", serviceString, printMenuItem(&serviceTxt[DEF_T_GRAM]));
            drawLine(serviceString, 3);
            snprintf(serviceString, sizeof(serviceString), "%2d ", 10 - i);
            drawLine(serviceString, 4);
        } while (oled.nextPage());
        delay(800);
    }

    oled.firstPage();
    do
    {
        snprintf(serviceString, sizeof(serviceString), "%s", printMenuItem(&serviceTxt[DEF_T_SETUP]));
        drawLine(serviceString, 2);
        snprintf(serviceString, sizeof(serviceString), "%s %1d", printMenuItem(&serviceTxt[DEF_T_SPOOL]), spool + 1);
        drawLine(serviceString, 3);
    } while (oled.nextPage());
    delay(800);

    offset_set_by_num(spool);

    WDT(WDTO_8S, 22);
}
#endif

#if SCALES_MODULE_NUM > 0 && AUTOPID_RUN == 0
void setSpool1()
{
    setSpool(0);
}
#endif

#if SCALES_MODULE_NUM > 1 && AUTOPID_RUN == 0
void setSpool2()
{
    setSpool(1);
}
#endif

#if SCALES_MODULE_NUM > 2 && AUTOPID_RUN == 0
void setSpool3()
{
    setSpool(2);
}
#endif

#if SCALES_MODULE_NUM > 3 && AUTOPID_RUN == 0
void setSpool4()
{
    setSpool(3);
}
#endif

#if SCALES_MODULE_NUM != 0 && AUTOPID_RUN == 0
void scaleShow()
{
    WDT(WDTO_500MS, 20);
    uint8_t stringNum = DEF_SCALES_COIL_1;
    oled.firstPage();
    do
    {
        for (uint8_t i = 0; i < SCALES_MODULE_NUM; i++)
        {
            snprintf(serviceString, sizeof(serviceString), "%10s %5d", printMenuItem(&menuTxt[stringNum + i * 3]), (uint16_t)hx711Multi.getMassMulti(i));
            drawLine(serviceString, i + 1, hx711Multi.getMassMulti(i) < FILAMENT_SENSOR_MASS, true, 0, 96);
        }
    } while (oled.nextPage());
    WDT_DISABLE();
}

void filamentCheck(uint8_t sensorNum, int16_t mass, stateS state, volatile uint16_t spoolMassArray[])
{
#ifdef FILAMENT_SENSOR_ON

    if (mass < 0)
    {
        filamentExpenseFlag[sensorNum] = ZERO;
    }

    switch (filamentExpenseFlag[sensorNum])
    {
    case ZERO:
        filamentExpenseFlag[sensorNum] = DATA_RESET;
        digitalWrite(FILAMENT_SENSOR, 0);
        break;
    case DATA_RESET:
        if (mass > ALERT_MASS)
        {
            filamentExpenseFlag[sensorNum] = UPDATE_DATA;
            eeprom_update_word((uint16_t *)&spoolMassArray[sensorNum], 0);
        }
        break;
    case UPDATE_DATA:
        eeprom_update_word((uint16_t *)&spoolMassArray[sensorNum], mass > 10 ? mass : 0);
        filamentExpenseFlag[sensorNum] = START;
        break;
    case START:
        if (eeprom_read_word((const uint16_t *)&spoolMassArray[sensorNum]) - mass >= FILAMENT_REFERENCE_FLOW_RATE_MASS)
        {
            filamentExpenseFlag[sensorNum] = WAIT_ALERT_MASS_1;
        }
        break;
    case WAIT_ALERT_MASS_1:
        if (mass < ALERT_MASS && mass > FILAMENT_SENSOR_MASS)
        {
            async_piii(ALERT_MASS_PIII_TIME * 1000);
            filamentExpenseFlag[sensorNum] = WAIT_ALERT_MASS_2;
        }
        break;
    case WAIT_ALERT_MASS_2:
        if (mass < FILAMENT_SENSOR_MASS)
        {
            digitalWrite(FILAMENT_SENSOR, 1);
            async_piii(FILAMENT_SENSOR_MASS_PIII_TIME * 1000);
            filamentExpenseFlag[sensorNum] = WAIT_RESET;
        }
        break;
        // case WAIT_RESET:
        //     if (mass < 0)
        //         filamentExpenseFlag[sensorNum] = ZERO;
        break;
    default:
        break;
    }
#endif
}

#endif

extern uint8_t sensor_temp_offset_table_eep[6]EEMEM {
    60,
    70,
    80,
    90,
    100,
    110,
};


void calibration()
{
    //Start
    for (size_t i = 0; i < SCALES_MODULE_NUM; i++)
    {
        hx711Multi.tempOffsetSetMulti(i, 0, 100);
    }

    PORTD |= (1 << DIMMER_PIN);
    fanMAX();

    // Греем
    while (iDryer.data.ntcTemp < 115 && iDryer.data.airTemp < 110)
    {
        WDT(WDTO_250MS, 32);
        iDryer.getData();
        WDT_DISABLE();
    }

    PORTD &= ~(1 << DIMMER_PIN);

    // Калибруем
    while(iDryer.data.airTemp > 45)
    {
        WDT(WDTO_8S, 33);
     
        iDryer.getData();
        uint8_t temp = (uint8_t)iDryer.data.ntcTemp;
        uint8_t air_temp = (uint8_t)iDryer.data.airTemp;
        uint8_t offset = 0;
    
        if (temp >= 60 && temp <= 110 && (temp % 10 == 0))
        {
            uint8_t idx = (temp - 60) / 10 + 1;
            uint8_t target = 60 + (idx - 1) * 10;
            int8_t offset = temp - air_temp;
            eeprom_write_byte(&sensor_temp_offset_table_eep[idx - 1], static_cast<uint8_t>(offset));
            
            for (size_t i = 0; i < SCALES_MODULE_NUM; i++)
            {
                hx711Multi.tempOffsetSetMulti(i, idx, 100);
            }
        }

        WDT_DISABLE();
    }
    fanOFF();
}

uint8_t temp_idx_rerurn(uint8_t temp)
{
    uint8_t idx = (temp - 60) / 10;
    return idx;
}