#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <U8g2lib.h>
#include <GyverTimers.h> // библиотека таймера
#include <GyverEncoder.h>
#include <GyverBME280.h>
#include <PID_v1.h>
#include <pidautotuner.h> //!! https://github.com/jackw01/arduino-pid-autotuner
#include <thermistor.h>
#include <Configuration.h>
#include "font.h"
#include "menu.h"
#include "def.h"
#include "HX711.h"
// #define KASYAK_FINDER

uint32_t ERROR_CODE EEMEM = 0x0; // EEMEM = 0b0000000000000000;
uint32_t WDT_ERROR = 0;          // EEMEM = 0b0000000000000000;

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
#define HEATER_MAX 9500
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
// Димер
#ifdef v220V
#define ZERO_PIN 2 // пин детектора нуля
#define INT_NUM 0  // соответствующий управляющему пину номер прерывания
#endif
#define DIMMER_PIN 5 // управляющий пин симистора или пин ШИМ

//!* 11,6,5,3
#define BUZZER_PIN 3
#define FAN FAN_PIN // TODO EXT обновить инструкцию
// #define EXT_PWM 11

#define SERVO_1_PIN 7
// #define SERVO_2_PIN 8

// ВЕСЫ
#define SCK_PIN A6
#define DT_PIN A7
// ВЕСЫ

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

#define MAX_ERROR 10

#define ADC_MIN 50
#define ADC_MAX 1024

#define OPEN 1
#define CLOSED 0

uint8_t lineHight = 16;
uint16_t dimmer = 0; // переменная диммера

#ifdef v220V
uint16_t lastDim;
#endif

stateS state = MENU;
uint8_t ERROR_COUNTER = 0;

uint8_t globalErrorFuncUUID = 0;
uint8_t errorTmpArray[32] = {0};

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

const byte CLK = A1;
const byte DT = A2;
const byte encBut = A3;
volatile static byte oldPorta;
volatile static byte PCMask;
volatile byte flagISR = 0;
// volatile byte intsFound = 0;
byte isr_1 = 0;
byte isr_2 = 0;
byte isr_3 = 0;

// uint8_t funcNum = 0;

uint8_t menuSize = 0;
// uint8_t settingsSize = 0;
// uint8_t buzzerAlarm = 0;
unsigned long oldTime = 0;
unsigned long oldTimer = 0;
// unsigned long servoOldTime1 = 0;
// unsigned long servoOldTime2 = 0;
// uint32_t timerOn = 0;
// uint32_t timerOff = 0;
// uint8_t servoState1 = CLOSED;
// uint8_t servoState2 = CLOSED;
unsigned long screenTime = 0;
char serviceString[30];

enum levelV
{
    NOTHING,
    DOWN,
    UP,
};

struct control // это могут быть и кнопки
{
    bool ok = 0;
    bool holded = 0;
    bool hold = 0;
    bool right = 0;
    bool left = 0;
};

struct subMenu
{
    bool changeVal;
    bool pointerUpdate;
    levelV levelUpdate = NOTHING;
    uint8_t level;    // текущий уровень меню
    uint8_t levelMax; // текущий уровень меню
    int8_t parentID;
    uint8_t membersQuantity;
    uint8_t membersID[25];
    uint8_t linesToScreen[SCREEN_LINES - MENU_HEADER];
    int8_t position;
    uint8_t pointerPos;
    int8_t min;
    int8_t max;
};

control controls;
subMenu subMenuM;

thermistor ntc(NTC_PIN, 0);
GyverBME280 bme;

#if SCREEN == 0
#define ERROR
#elif SCREEN == 1
U8G2_SH1106_128X64_NONAME_1_HW_I2C oled(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
#elif SCREEN == 2
U8G2_SSD1306_128X64_NONAME_1_HW_I2C oled(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
#endif

Encoder enc(CLK, DT, encBut, MY_ENCODER_TYPE);

// double mainSetpoint;
double Setpoint, Input, Output;

#ifdef v24v
PID pid(&Input, &Output, &Setpoint, 2, 1, 5, DIRECT);
#else
PID pid(&Input, &Output, &Setpoint, 2, 1, 5, DIRECT);
#endif

struct Data
{
    double ntcTemp = 0;
    float bmeTemp = 0;
    float bmeHumidity = 0;
    unsigned long startTime = 0;
    // unsigned long errorTime26 = 0;
    // unsigned long errorTime27 = 0;
    unsigned long errorTime28 = 0;
    // unsigned long errorTime29 = 0;
    uint8_t setTemp = 0;
    uint8_t setHumidity = 0;
    uint16_t setTime = 0;
    uint8_t flag = 0;
    uint8_t flagScreenUpdate = 0;
    uint8_t flagTimeCounter = 0;
    uint8_t setFan = 0;
    double Kp = 0.0;
    double Ki = 0.0;
    double Kd = 0.0;
    uint16_t sampleTime = 0;
    uint8_t deltaT = 0;

    bool operator!=(const Data &other) const
    {
        if (int(this->ntcTemp) != int(other.ntcTemp) || int(this->bmeTemp) != int(other.bmeTemp) || int(this->bmeHumidity) != int(other.bmeHumidity)
            // || int(this->startTime) == int(other.startTime)
            // || int(this->setTemp) == int(other.setTemp)
            // || int(this->setHumidity) == int(other.setHumidity)
            // || int(this->setTime) == int(other.setTime)
            // || int(this->setFan) == int(other.setFan)
            // || int(this->Kp) == int(other.Kp)
            // || int(this->Ki) == int(other.Ki)
            // || int(this->Kd) == int(other.Kd)
        )
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    //   bool operator!=(const Data &other) const {
    //     return !(*this == other);
    //   }
};

/* 01 */ void heaterOFF();
/* 02 */ void heaterON(uint16_t Output, uint16_t &dimmer);
/* 03 */ void servoPulse(int pin, int angle);
/* 04 */ void updateIDyerData();
/* 05 */ void encoderSate(struct control *control);
/* 06 */ void screen(struct subMenu *subMenu);
/* 07 */ void controlsHandler(const menuS constMenu[], uint16_t editableMenu[], const ptrFunc functionMenu[], struct control *encoder, struct subMenu *subMenu);
/* 08 */ void submenuHandler(const menuS constMenu[], uint8_t menuSize, struct subMenu *subMenu);
/* 09 */ void piii(uint16_t time_ms);
/* 10 */ void dryStart();
/* 11 */ void storageStart();
/* 12 */ void autoPidM();
/* 13 */ uint32_t printError(uint32_t error);
/* 14 */ uint32_t readError();
/* 15 */ bool setError(uint8_t errorCode);
/* 16 */ void displayPrint(struct subMenu *subMenu);
/* 17 */ void displayPrintMode();
         void pwm_test();
/* 18 */ // PID TUNING;
/* 20 */ // CASE OFF
/* 21 */ // CASE ON
/* 22 */ // CASE MENU
/* 23 */ // CASE DRY
/* 24 */ // CASE STORAGE
/* 25 */ // CASE AUTOPID

/* 26 */ // NTC MIN
/* 27 */ // NTC MAX
/* 28 */ // BME MIN
/* 29 */ // BME MAX

/* 30 */ // ADC ERROR
/* 0 */  // ADC ACCUMULATED ERROR

/* 31 */ // iDryer.getData

void WDT(uint16_t time, uint8_t current_function_uuid);
void WDT_DISABLE();

class iDryer
{
public:
    Data data;
    Data oldData;
    bool getData()
    {
        data.ntcTemp = (ntc.analog2temp() + data.ntcTemp) / 2.0;
        data.bmeTemp = (bme.readTemperature() + data.bmeTemp) / 2.0;
        data.bmeHumidity = (bme.readHumidity() + data.bmeHumidity) / 2.0;
        if (data != oldData && millis() - screenTime > SCREEN_UPADATE_TIME)
        {
            screenTime = millis();
            data.flagScreenUpdate = 1;
            oldData = data;
        }
        else
        {
            data.flagScreenUpdate = 0;
        }

        if (data.bmeTemp > data.setTemp && !data.flagTimeCounter)
        {
            data.flagTimeCounter = true;
        }

        // if (data.ntcTemp < TMP_MIN ||
        //     data.ntcTemp > TMP_MAX + 10 ||
        //     data.bmeTemp < TMP_MIN ||
        //     data.bmeTemp > TMP_MAX + 10)
        // {
        // return false;
        // }

        if (data.ntcTemp < TMP_MIN)
        {
            setError(26);
#ifdef KASYAK_FINDER
            Serial.println(26);
#endif
            return false;
        }
        if (data.ntcTemp > TMP_MAX + 10)
        {
            setError(27);
#ifdef KASYAK_FINDER
            Serial.println(27);
#endif
            return false;
        }
        //!! Костыль, но нужный
        if (data.bmeTemp < TMP_MIN)
        {
            if (millis() - data.errorTime28 > 1000)
                data.errorTime28 = millis();

            if (millis() - data.errorTime28 > 600 && millis() - data.errorTime28 < 1000)
            {
                setError(28);
#ifdef KASYAK_FINDER
                Serial.println(28);
#endif
                return false;
            }
        }
        if (data.bmeTemp > TMP_MAX + 10)
        {
            setError(29);
#ifdef KASYAK_FINDER
            Serial.println(28);
#endif
            return false;
        }
        return true;
    }

} iDryer;

#ifdef WITH_BLACKJACK_AND_HOOKERS
class servo
{
public:
#define OPEN 1
#define CLOSED 0
    uint8_t pin = 0;
    unsigned long oldTime = 0;
    uint16_t openTime = 0;
    uint16_t closedTime = 0;
    uint8_t angle = 0;
    uint8_t state = CLOSED;
    servo(uint8_t _srvPin, uint16_t _closedTime, uint16_t _openTime, uint16_t _angle)
    {
        pin = _srvPin;
        closedTime = _closedTime;
        openTime = _openTime;
        angle = _angle;
        // close();
    }
    void set(uint16_t _closedTime, uint16_t _openTime, uint16_t _angle)
    {
        closedTime = _closedTime;
        openTime = _openTime;
        angle = _angle;
    }
    void close()
    {
            if (oldTime < millis())
            {
                servoPulse(pin, 90);
                state = CLOSED;
                oldTime = millis() + (unsigned long)closedTime * 1000UL * 60UL;
            }
    }
    void test()
    {
        servoPulse(pin, 90);
        state = CLOSED;
        if (state == CLOSED)
        {
            servoPulse(pin, 90 + angle);
            state = OPEN;
        }
        if (state == OPEN)
        {
            servoPulse(pin, 90);
            state = CLOSED;
        }
        oldTime = millis();
    }
    void check()
    {
        if (state == CLOSED)
        {
            if (oldTime < millis())
            {
                servoPulse(pin, 90 + angle);
                state = OPEN;
                oldTime = millis() + (unsigned long)openTime * 1000UL * 60UL;
            }
        }
        if (state == OPEN)
        {
            if (oldTime < millis())
            {
                servoPulse(pin, 90);
                state = CLOSED;
                oldTime = millis() + (unsigned long)closedTime * 1000UL * 60UL;
            }
        }
    }

    void servoPulse(int pin, int angle)
    {
        WDT(WDTO_4S, 3);
        // piii(SERVO_CUCKOO);
        int pulseWidth = map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
        for (int pulseCounter = 0; pulseCounter <= 20; pulseCounter++)
        {
            unsigned long startTime = micros();
            digitalWrite(pin, HIGH);
            while (micros() - startTime < pulseWidth)
            {
            }
            digitalWrite(pin, LOW);
            while (micros() - startTime < SERVO_PERIOD_MS * 1000 - pulseWidth)
            {
            }
        }
        WDT_DISABLE();
    }
};

servo Servo1(SERVO_1_PIN, eeprom_read_word(&menuVal[DEF_SERVO1_CLOSED]), eeprom_read_word(&menuVal[DEF_SERVO1_OPEN]), eeprom_read_word(&menuVal[DEF_SERVO1_CORNER]));
// servo Servo2(SERVO_2_PIN, eeprom_read_word(&menuVal[DEF_SERVO2_CLOSED]), eeprom_read_word(&menuVal[DEF_SERVO2_OPEN]), eeprom_read_word(&menuVal[DEF_SERVO2_CORNER]));
void servoTest()
{   
    updateIDyerData();
    Servo1.test();
}


#endif

#ifdef v220V
void isr()
{
#ifdef DEBUG
    testTIMER_COUNT++;
#endif
    if (state == DRY || state == STORAGE || state == AUTOPID)
    {
        if (lastDim != dimmer)
        {
            Timer1.setPeriod(lastDim = dimmer);
        }
        else
        {
            Timer1.restart();
        }
        Timer1.setPeriod(dimmer);
    }
}

ISR(TIMER1_A)
{
    if (dimmer) digitalWrite(DIMMER_PIN, 1);
    digitalWrite(DIMMER_PIN, 0);
    Timer1.setPeriod(20000);
    // Timer1.stop();
}

#endif

ISR(PCINT1_vect)
{
    byte change, v1, v2, v3;
    change = oldPorta ^ PINC;

    v1 = oldPorta & (1 << PCINT9);
    v2 = oldPorta & (1 << PCINT10);
    v3 = oldPorta & (1 << PCINT11);

    if (v1 == 0 && change & (1 << PCINT9))
        isr_1 = 1;
    if (v2 == 0 && change & (1 << PCINT10))
        isr_2 = 1;
    if (v3 == 0 && change & (1 << PCINT11))
        isr_3 = 1;
    if (change && (v1 == 0 || v2 == 0 || v3 == 0))
    {
        flagISR = 1;
        isr_1 = isr_2 = isr_3 = 0;
        // enc.tick();
    }
    oldPorta = PINC;
}

char *printMenuItem(PGM_P const *text) // печать строки из prm
{
    static char buffer[22]; // TODO Проверить 11 символов на экране 20/22
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
    WDT(WDTO_4S, 16);
#ifdef DEBUG
    wdt_reset();
    wdt_disable();
#endif
    oled.firstPage();
    do
    {
        oled.setFont(u8g2_font);
        uint8_t maxPos = SCREEN_LINES - MENU_HEADER < subMenu->membersQuantity ? SCREEN_LINES - MENU_HEADER : subMenu->membersQuantity;
        oled.drawUTF8((128 - oled.getUTF8Width(printMenuItem(&menuTxt[subMenuM.parentID]))) / 2 - 10, 16, printMenuItem(&menuTxt[subMenuM.parentID]));
        if (subMenuM.parentID == 0)
        {
            char val[6];
            sprintf(val, "%2hu/%2hu", (uint16_t)iDryer.data.bmeHumidity, (uint16_t)iDryer.data.bmeTemp);
#ifdef DEBUG
            sprintf(val, "%4d", oldTime1 - oldTime2);
#endif
            oled.drawUTF8(88, 16, val);
            sprintf(val, "%2hu", (uint16_t)iDryer.data.ntcTemp);
#ifdef DEBUG
            sprintf(val, "%3hu", dimmer);
#endif
            oled.drawUTF8(0, 16, val);
        }
        oled.drawLine(0, lineHight + 2, 128, lineHight + 2);
        for (uint8_t i = 0; i < maxPos; i++)
        {
            oled.drawUTF8(0, (i + 2) * lineHight, printMenuItem(&menuTxt[subMenuM.membersID[subMenuM.linesToScreen[i]]]));

            if (pgm_read_word(&menuPGM[subMenuM.membersID[subMenuM.linesToScreen[i]]].min) ||
                pgm_read_word(&menuPGM[subMenuM.membersID[subMenuM.linesToScreen[i]]].max) ||
                eeprom_read_word(&menuVal[subMenuM.membersID[subMenuM.linesToScreen[i]]]))
            {
                char val[5];
                sprintf(val, "%4hu", eeprom_read_word(&menuVal[subMenuM.membersID[subMenuM.linesToScreen[i]]]));
                oled.drawUTF8(95, (i + 2) * lineHight, val);
            }

            if (subMenuM.pointerPos == i)
            {
                if (!subMenu->changeVal)
                {
                    oled.drawButtonUTF8(0, lineHight * (i + 2) - 0, U8G2_BTN_INV, 128, 0, 0, "");
                }
                else
                {
                    oled.drawButtonUTF8(96, lineHight * (i + 2) - 0, U8G2_BTN_INV, 128, 0, 0, "");
                }
            }
        }
    } while (oled.nextPage());
    iDryer.data.flagScreenUpdate = 0;
    WDT_DISABLE();
}

void displayPrintMode()
{
    WDT(WDTO_4S, 17);
    ERROR_CODE = 0;
    oled.firstPage();
    do
    {
        oled.setFont(u8g2_font);

        uint8_t text = 0;
        if (state == DRY)
        {
            text = DEF_MENU_DRYING;
        }
        if (state == STORAGE)
        {
            text = DEF_MENU_STORAGE;
        }

        char val[4];
      oled.drawUTF8((128 - oled.getUTF8Width(printMenuItem(&menuTxt[text]))) / 2, lineHight, printMenuItem(&menuTxt[text]));
        // sprintf(val, "%2hu", Setpoint);
        sprintf(val, "%2hu", iDryer.data.setTemp);
        oled.drawUTF8(0, lineHight, val);
        text == DEF_MENU_DRYING ? sprintf(val, "%3hu", iDryer.data.setTime) : sprintf(val, "%3hu", iDryer.data.setHumidity);
        oled.drawUTF8(100, lineHight, val);
        oled.drawButtonUTF8(0, lineHight - 1, U8G2_BTN_INV, 128, 1, 1, "");

        for (uint8_t i = 0; i < 3; i++)
        {
            oled.drawUTF8(0, (i + 2) * lineHight, printMenuItem(&serviceTxt[i + 6]));
            text = i;
            uint8_t data = 0;
            switch (i)
            {
            case 0: // Воздух
                data = iDryer.data.bmeTemp;
                break;
            case 1: // Нагреватель
                data = iDryer.data.ntcTemp;
                break;
            case 2: // Влажность
                data = iDryer.data.bmeHumidity;
                break;
            default:
                break;
            }
            sprintf(val, "%3hu", data);
            oled.drawUTF8(100, (i + 2) * lineHight, val);
        }
    } while (oled.nextPage());
    iDryer.data.flagScreenUpdate = 0;
    WDT_DISABLE();
}

void setup()
{
#ifdef KASYAK_FINDER
    Serial.begin(9600);
#endif
    oldPorta = PINC;
    PCMSK1 |= (1 << PCINT9);
    PCMSK1 |= (1 << PCINT10);
    PCMSK1 |= (1 << PCINT11);
    // Serial.println(PCMSK1,HEX);
    PCMask = PCMSK1;
    PCICR |= (1 << PCIE1);


#ifdef v220V
    dimmer = HEATER_MAX;

    pinMode(ZERO_PIN, INPUT_PULLUP);
    pinMode(DIMMER_PIN, OUTPUT);
    // attachInterrupt(INT_NUM, isr, RISING);
    // Timer1.enableISR();
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
    pinMode(SERVO_1_PIN, OUTPUT);
    digitalWrite(SERVO_1_PIN, 0);
    // pinMode(SERVO_2_PIN, OUTPUT);
    // digitalWrite(SERVO_2_PIN, 0);

    oled.begin();
    oled.setFlipMode(SCREEN_FLIP);
    oled.enableUTF8Print();
    oled.setFont(u8g2_font);

    menuSize = sizeof(menuTxt) / sizeof(menuTxt[0]);
    uint32_t errorCode = readError();

    if (errorCode)
    {
        do
        {
            oled.setFont(u8g2_font);
            oled.drawUTF8((128 - oled.getUTF8Width(printMenuItem(&serviceTxt[DEF_T_ERROR]))) / 2, lineHight, printMenuItem(&serviceTxt[DEF_T_ERROR]));
            uint8_t charCounter = 0;
            for (uint8_t i = 0; i < 32; i++)
            {
                if (errorTmpArray[i])
                {
                    sprintf(serviceString, "%2d", i);
                    oled.drawUTF8((charCounter * 32) % 128 + 4, lineHight * (charCounter / 4 + 2), serviceString);
                    if (charCounter % 5 == 0)
                        sprintf(serviceString, "");
                    charCounter++;
                }
            }
            oled.drawButtonUTF8(0, 1 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
            oled.drawButtonUTF8(0, 2 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
            oled.drawButtonUTF8(0, 3 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
            oled.drawButtonUTF8(0, 4 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
        } while (oled.nextPage());
        MCUSR = 0;
        analogWrite(FAN, 200);
        while (digitalRead(encBut))
        {
            piii(250);
            delay(250);
        }
        eeprom_write_dword(&ERROR_CODE, 0UL);
    }

    uint16_t test;
    test = eeprom_read_word(&menuVal[0]);
    if (test != 123)
    {
#ifdef WITH_BLACKJACK_AND_HOOKERS
#else
        eeprom_write_block(&menuValDefault, &menuVal, sizeof(menuValDefault));
        for (uint8_t i = 0; i < sizeof(menuValDefault) / sizeof(menuValDefault[0]); i++)
        {
            uint16_t val;
            val = eeprom_read_word((uint16_t *)&menuVal[i]);
            // Serial.println("menuValDefault 568");
            // Serial.print(i);
            // Serial.print(": ");
            // Serial.println(val);
        }
#endif
    }


#ifdef WITH_BLACKJACK_AND_HOOKERS
#if OVERWRITE_PID == 1
    eeprom_update_word(&menuVal[DEF_PID_KP], K_PROPRTIONAL);
    eeprom_update_word(&menuVal[DEF_PID_KI], K_INTEGRAL);
    eeprom_update_word(&menuVal[DEF_PID_KD], K_DERIVATIVE);
    eeprom_update_word(&menuVal[DEF_AVTOPID_TIME_MS], K_SAMPLE_TIME);
#endif

#else
#if OVERWRITE_PID == 1
    menuValDefault[DEF_PID_KP] = K_PROPRTIONAL;
    menuValDefault[DEF_PID_KI] = K_INTEGRAL;
    menuValDefault[DEF_PID_KD] = K_DERIVATIVE;
    menuValDefault[DEF_AVTOPID_TIME_MS] = K_SAMPLE_TIME;
#endif
#endif

    updateIDyerData();

    pid.SetMode(AUTOMATIC); //
    pid.SetControllerDirection(DIRECT); // REVERSE
    pid.SetOutputLimits(HEATER_MIN, HEATER_MAX);
    pid.SetTunings(double(iDryer.data.Kp), double(iDryer.data.Ki), double(iDryer.data.Kd), P_ON_E);
    pid.SetSampleTime(iDryer.data.sampleTime);


    oled.firstPage();
    do
    {
        oled.drawUTF8((128 - oled.getUTF8Width(printMenuItem(&serviceTxt[DEF_T_AYRA]))) / 2, lineHight, printMenuItem(&serviceTxt[DEF_T_AYRA]));
        oled.drawUTF8((128 - oled.getUTF8Width(printMenuItem(&serviceTxt[DEF_T_VERSION]))) / 2, lineHight * 2, printMenuItem(&serviceTxt[DEF_T_VERSION]));
        oled.drawUTF8((128 - oled.getUTF8Width(printMenuItem(&serviceTxt[DEF_T_VER]))) / 2, lineHight * 3, printMenuItem(&serviceTxt[DEF_T_VER]));
        // if (errorCode)
        // {
        //     {
        //         sprintf(serviceString, "%8s %hu", printMenuItem(&serviceTxt[DEF_T_ERROR]), errorCode);
        //         oled.drawUTF8((128 - oled.getUTF8Width(serviceString)) / 2, lineHight * 4, serviceString);
        //         oled.drawButtonUTF8(0, lineHight * 4, U8G2_BTN_INV, 128, 1, 1, "");
        //     }
        // }
    } while (oled.nextPage());

    // delay(1000);

    enc.setTickMode(MANUAL);
    enc.setPinMode(0);    // HIGH_PULL
    enc.setBtnPinMode(0); // HIGH_PULL

    if (!bme.begin(0x76))
    {
        // Serial.println("Could not find a valid BME280 sensor, check wiring!");
        // while (1)
        ;
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

#ifdef WITH_BLACKJACK_AND_HOOKERS

    Servo1.test();
    // Servo2.test();
#endif

#ifdef PWM_TEST
    pwm_test();
#endif
}

void loop()
{
    // analogWrite(FAN, 100);
    enc.tick();
    uint16_t tmpTemp = analogRead(NTC_PIN);
    if (tmpTemp <= ADC_MIN || tmpTemp >= ADC_MAX)
    {
        if(state == DRY || state == STORAGE)
        {
            setError(30);
            state = NTC_ERROR;
        }
    }

    if (enc.isHold() && (state == DRY || state == STORAGE))
    {
        WDT_DISABLE();

        state = MENU;

        heaterOFF();
        piii(500);
        subMenuM.levelUpdate = UP;
        subMenuM.parentID = 0;
        subMenuM.pointerPos = 0;
        subMenuM.pointerUpdate = 1;
        while (digitalRead(encBut))
        {
            ;
        }
        enc.resetStates();
    }

    switch (state)
    {
    case NTC_ERROR:
        analogWrite(FAN, 255);
        heaterOFF();
#ifdef KASYAK_FINDER
        Serial.println("NTC ERROR");
#endif
        oled.clear();
        oled.firstPage();
        do
        {
            oled.setFont(u8g2_font);
            oled.drawUTF8((128 - oled.getUTF8Width(printMenuItem(&serviceTxt[DEF_T_ERROR]))) / 2, lineHight, printMenuItem(&serviceTxt[DEF_T_ERROR]));
            oled.drawUTF8((128 - oled.getUTF8Width(printMenuItem(&serviceTxt[DEF_T_CHECK]))) / 2, lineHight * 2, printMenuItem(&serviceTxt[DEF_T_CHECK]));
            oled.drawUTF8((128 - oled.getUTF8Width(printMenuItem(&serviceTxt[DEF_T_THERMISTOR]))) / 2, lineHight * 3, printMenuItem(&serviceTxt[DEF_T_THERMISTOR]));
            oled.drawUTF8((128 - oled.getUTF8Width(printMenuItem(&serviceTxt[DEF_T_OVERLOAD]))) / 2, lineHight * 4, printMenuItem(&serviceTxt[DEF_T_OVERLOAD]));
            oled.drawButtonUTF8(0, 1 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
            oled.drawButtonUTF8(0, 2 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
            oled.drawButtonUTF8(0, 3 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
            oled.drawButtonUTF8(0, 4 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
        } while (oled.nextPage());

        while (digitalRead(encBut))
        {
            piii(250);
            delay(250);
#ifdef KASYAK_FINDER
            Serial.println("NTC ERROR PIII");
#endif
        }
        break;
    case OFF:
        // WDT(WDTO_1S, 20);
        // WDT_DISABLE();
        break;
    case ON:
        // WDT(WDTO_1S, 21);
        // WDT_DISABLE();
        break;
    case MENU:
        WDT(WDTO_4S, 22);
        if (iDryer.getData())
        {
            if (iDryer.data.ntcTemp > 40)
            {
                analogWrite(FAN, 255);
            }
            else
            {
                analogWrite(FAN, 0);
            }
        }

        if (flagISR)
        {
            flagISR = 0;
        }
        encoderSate(&controls);
        controlsHandler(menuPGM, menuVal, menuFunc, &controls, &subMenuM);
        if (subMenuM.levelUpdate)
        {
            submenuHandler(menuPGM, menuSize, &subMenuM);
            screen(&subMenuM); //!! добавить subMenuM.pointerUpdate  в submenuHandler
            displayPrint(&subMenuM);
        }
        if (subMenuM.pointerUpdate)
        {
            screen(&subMenuM);
            displayPrint(&subMenuM);
        }
        break;
    case DRY:
        WDT(WDTO_8S, 23);
        if (iDryer.getData())
        {
            if (iDryer.data.flagScreenUpdate)
                displayPrintMode();
        }
        else
        {
            ERROR_COUNTER++;
#ifdef KASYAK_FINDER
            Serial.println("DRY ERROR COUNTER");
#endif
            if (ERROR_COUNTER > MAX_ERROR)
            {
                setError(0);
#ifdef KASYAK_FINDER
                Serial.println("DRY ERROR COUNTER SET 0");
#endif
                state = NTC_ERROR;
            }
        }

        iDryer.data.flagTimeCounter ? analogWrite(FAN, map(iDryer.data.setFan, 0, 100, 0, 255)) : analogWrite(FAN, 255);

        if (iDryer.data.bmeTemp < iDryer.data.setTemp)
        {
            Setpoint = iDryer.data.setTemp + iDryer.data.deltaT;
        }
        else
        {
            Setpoint = iDryer.data.setTemp - iDryer.data.bmeTemp + iDryer.data.setTemp;
            if (Setpoint > iDryer.data.setTemp + iDryer.data.deltaT)
                Setpoint = iDryer.data.setTemp + iDryer.data.deltaT;
        }

        if (Setpoint > TMP_MAX)
            Setpoint = TMP_MAX;

        Input = double(iDryer.data.ntcTemp);
        pid.Compute();

        heaterON(Output, dimmer);

        if (millis() - oldTimer >= 60000 && iDryer.data.flagTimeCounter)
        {
            oldTimer = millis();
            iDryer.data.setTime--;
        }
        if (iDryer.data.setTime == 0)
        {
            subMenuM.parentID = 5; // ID пункта хранение
            heaterOFF();
            WDT_DISABLE();
            storageStart();
        }
#ifdef WITH_BLACKJACK_AND_HOOKERS
        Servo1.check();
        // Servo2.check();
#endif
        break;
#ifndef KASYAK_FINDER
    case STORAGE:
        WDT(WDTO_8S, 24);
        if (iDryer.getData())
        {
            if (iDryer.data.flagScreenUpdate)
                displayPrintMode();
        }
        else
        {
            ERROR_COUNTER++;
            if (ERROR_COUNTER > MAX_ERROR)
            {
                setError(31);
                state = NTC_ERROR;
            }
        }

        if (iDryer.data.bmeTemp < iDryer.data.setTemp)
        {
            Setpoint = iDryer.data.setTemp + iDryer.data.deltaT;
        }
        else
        {
            Setpoint = iDryer.data.setTemp - iDryer.data.bmeTemp + iDryer.data.setTemp;
            if (Setpoint > iDryer.data.setTemp + iDryer.data.deltaT)
                Setpoint = iDryer.data.setTemp + iDryer.data.deltaT;
        }

        if (Setpoint > TMP_MAX)
            Setpoint = TMP_MAX;

        if (iDryer.data.bmeHumidity > iDryer.data.setHumidity && iDryer.data.flag)
        // if ((iDryer.data.bmeHumidity > iDryer.data.setHumidity && iDryer.data.flag) ||
        //     (iDryer.data.bmeHumidity > iDryer.data.setHumidity + 2 && !iDryer.data.flag))
        {
            // iDryer.data.flag = 1;
            Input = double(iDryer.data.ntcTemp);
            pid.Compute();

            iDryer.data.flagTimeCounter ? analogWrite(FAN, map(iDryer.data.setFan, 0, 100, 0, 255)) : analogWrite(FAN, 255);
            heaterON(Output, dimmer);
#ifdef WITH_BLACKJACK_AND_HOOKERS
            Servo1.check();
            // Servo2.check();
#endif
        }
        else
        {
            iDryer.data.flag = 0;
            Input = double(iDryer.data.ntcTemp);
            // Setpoint = 0;
            // pid.Compute();
            // heaterON(Output = 0, dimmer = HEATER_MAX);
            if (iDryer.data.ntcTemp < iDryer.data.setTemp + 5)
            {
                analogWrite(FAN, 0);
            }
            heaterOFF();
            if (Servo1.state) Servo1.close();
            // if (Servo2.state) Servo2.close();
        }

        if (iDryer.data.bmeHumidity > iDryer.data.setHumidity + 2 && !iDryer.data.flag)
        {
            iDryer.data.flag = 1;
#ifdef v220V
            attachInterrupt(INT_NUM, isr, RISING);
            Timer1.enableISR(CHANNEL_A);
            // Timer1.enableISR(); //TODO Timer
#else
#endif
        }

        break;
    case AUTOPID:
        WDT(WDTO_4S, 25);
        PIDAutotuner tuner = PIDAutotuner();
        tuner.setTargetInputValue((float)iDryer.data.setTemp);
        tuner.setLoopInterval(long(iDryer.data.sampleTime) * 1000);
        tuner.setTuningCycles(AUTOPID_ATTEMPT);
        tuner.setOutputRange(HEATER_MIN, HEATER_MAX);
        tuner.setZNMode(PIDAutotuner::ZNModeNoOvershoot); // ZNModeNoOvershoot - Defaults,   ZNModeBasicPID
        tuner.startTuningLoop(micros());

        oled.clear();
        oled.firstPage();
        do
        {
            oled.setFont(u8g2_font);
            oled.drawUTF8((128 - oled.getUTF8Width(printMenuItem(&menuTxt[DEF_PID_AVTOPID]))) / 2, lineHight * 3, printMenuItem(&menuTxt[DEF_PID_AVTOPID]));
            oled.drawButtonUTF8(0, 1 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
            oled.drawButtonUTF8(0, 2 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
            oled.drawButtonUTF8(0, 3 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
            oled.drawButtonUTF8(0, 4 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
        } while (oled.nextPage());
        piii(500);
        delay(500);

        unsigned long microseconds;

        analogWrite(FAN, 255);
        while (!tuner.isFinished())
        {
            WDT(WDTO_4S, 18);
            microseconds = micros();
            iDryer.getData();

            heaterON((uint16_t)(tuner.tunePID(double(iDryer.data.ntcTemp), microseconds)), dimmer);

            oled.firstPage();
            do
            {
                oled.setFont(u8g2_font);
                sprintf(serviceString, "ТСТ-%d/%d  %dС", tuner.getCycle(), AUTOPID_ATTEMPT, (uint8_t)ntc.analog2temp());
                oled.drawUTF8((128 - oled.getUTF8Width(serviceString)) / 2, lineHight * 1, serviceString);
                sprintf(serviceString, "%4s %6hu", printMenuItem(&menuTxt[DEF_PID_KP]), (uint16_t)(abs(tuner.getKp()) * 100));
                oled.drawUTF8((128 - oled.getUTF8Width(serviceString)) / 2, lineHight * 2, serviceString);
                sprintf(serviceString, "%4s %6hu", printMenuItem(&menuTxt[DEF_PID_KI]), (uint16_t)(abs(tuner.getKi()) * 100));
                oled.drawUTF8((128 - oled.getUTF8Width(serviceString)) / 2, lineHight * 3, serviceString);
                sprintf(serviceString, "%2s %6hu", printMenuItem(&menuTxt[DEF_PID_KD]), (uint16_t)(abs(tuner.getKd()) * 100));
                oled.drawUTF8((128 - oled.getUTF8Width(serviceString)) / 2, lineHight * 4, serviceString);
            } while (oled.nextPage());

            while (micros() - microseconds < (unsigned long)iDryer.data.sampleTime * 1000)
            {
            }
            WDT_DISABLE();
        }

        oled.firstPage();
        do
        {
            oled.setFont(u8g2_font);
            sprintf(serviceString, "ТСТ-%d/%d  %dС", tuner.getCycle(), AUTOPID_ATTEMPT, (uint8_t)ntc.analog2temp());
            oled.drawUTF8((128 - oled.getUTF8Width(serviceString)) / 2, lineHight * 1, serviceString);
            sprintf(serviceString, "%4s %6hu", printMenuItem(&menuTxt[DEF_PID_KP]), (uint16_t)(abs(tuner.getKp()) * 100));
            oled.drawUTF8((128 - oled.getUTF8Width(serviceString)) / 2, lineHight * 2, serviceString);
            sprintf(serviceString, "%4s %6hu", printMenuItem(&menuTxt[DEF_PID_KI]), (uint16_t)(abs(tuner.getKi()) * 100));
            oled.drawUTF8((128 - oled.getUTF8Width(serviceString)) / 2, lineHight * 3, serviceString);
            sprintf(serviceString, "%2s %6hu", printMenuItem(&menuTxt[DEF_PID_KD]), (uint16_t)(abs(tuner.getKd()) * 100));
            oled.drawUTF8((128 - oled.getUTF8Width(serviceString)) / 2, lineHight * 4, serviceString);
        } while (oled.nextPage());

        heaterOFF();
        analogWrite(FAN, 250);

        delay(1000);

        eeprom_update_word(&menuVal[DEF_PID_KP], (uint16_t)(abs(tuner.getKp()) * 100));
        eeprom_update_word(&menuVal[DEF_PID_KI], (uint16_t)(abs(tuner.getKi()) * 100));
        eeprom_update_word(&menuVal[DEF_PID_KD], (uint16_t)(abs(tuner.getKd()) * 100));
        eeprom_update_word(&menuVal[DEF_SETTINGS_DELTA], iDryer.data.ntcTemp - iDryer.data.bmeTemp);
        // saveAll();
        updateIDyerData();

        oled.clear();
        do
        {
            oled.setFont(u8g2_font);
            oled.drawUTF8((128 - oled.getUTF8Width(printMenuItem(&menuTxt[DEF_SETTINGS_PID]))) / 2, lineHight * 2, printMenuItem(&menuTxt[DEF_SETTINGS_PID]));
            oled.drawUTF8((128 - oled.getUTF8Width(printMenuItem(&serviceTxt[5]))) / 2, lineHight * 3, printMenuItem(&serviceTxt[5]));
        } while (oled.nextPage());

        uint8_t time_delay = 40;
        while (time_delay)
        {
            char val[4];
            oled.firstPage();
            do
            {
                sprintf(val, "%3d", time_delay);
                oled.setFont(u8g2_font);
                sprintf(serviceString, "%12s", printMenuItem(&serviceTxt[4]));
                oled.drawUTF8((128 - oled.getUTF8Width(printMenuItem(&serviceTxt[4]))) / 2, lineHight * 2, printMenuItem(&serviceTxt[4]));

                oled.drawUTF8((128 - oled.getUTF8Width(val)) / 2, lineHight * 3, val);
            } while (oled.nextPage());
            time_delay--;
            delay(1000);
        }

        oled.clear();

        wdt_disable();

        subMenuM.levelUpdate = DOWN;
        subMenuM.pointerUpdate = 1;
        subMenuM.parentID = 0;
        subMenuM.position = 0;
        memset(subMenuM.membersID, 0, sizeof(subMenuM.membersID) / sizeof(subMenuM.membersID[0]));

        // iDryer.data.flagScreenUpdate = 1;
        state = MENU;
        break;
#endif
    }
}

// TODO запилить нормальный энкодер
void encoderSate(struct control *control)
{
    WDT(WDTO_120MS, 5);
    // enc.tick();
    if (enc.isClick())
    {
        control->ok = 1;
    }
    if (enc.isHolded())
    {
        control->hold = 1;
    }
    if (enc.isRelease())
    {
        enc.resetStates();
    }
    if (enc.isLeft())
    {
        control->left = 1;
    }
    if (enc.isRight())
    {
        control->right = 1;
    }
    WDT_DISABLE();
}

void controlsHandler(const menuS constMenu[], uint16_t editableMenu[], const ptrFunc functionMenu[], struct control *encoder, struct subMenu *subMenu)
{
    WDT(WDTO_250MS, 7);
    if (encoder->ok)
    {
        encoder->ok = false;
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

    if (encoder->hold)
    {
        encoder->hold = false;
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
#ifdef ENCODER_REVERSE
    if (encoder->left)
    {
        encoder->left = false;
#else
    if (encoder->right) // Timer1.enableISR();
    {
        encoder->right = false;
#endif
        if (!subMenu->changeVal)
        {
            subMenu->position == subMenu->max ? subMenu->position = 0 : subMenu->position++;
        }
        else
        {
            if (eeprom_read_word(&editableMenu[subMenuM.membersID[subMenuM.linesToScreen[subMenuM.pointerPos]]]) <
                pgm_read_word(&constMenu[subMenuM.membersID[subMenuM.linesToScreen[subMenuM.pointerPos]]].max))
            {
                uint16_t val = eeprom_read_word(&editableMenu[subMenuM.membersID[subMenuM.linesToScreen[subMenuM.pointerPos]]]);
                val++;
                eeprom_write_word((uint16_t *)&editableMenu[subMenuM.membersID[subMenuM.linesToScreen[subMenuM.pointerPos]]], val);
            }
        }
        subMenu->pointerUpdate = true;
    }

#ifdef ENCODER_REVERSE
    if (encoder->right)
    {
        encoder->right = false;
#else
    if (encoder->left) // Timer1.enableISR();
    {
        encoder->left = false;
#endif
        if (!subMenu->changeVal)
        {
            subMenu->position == subMenu->min ? subMenu->position = subMenu->max : subMenu->position--;
        }
        else
        {
            if (eeprom_read_word(&editableMenu[subMenuM.membersID[subMenuM.linesToScreen[subMenuM.pointerPos]]]) >
                pgm_read_word(&constMenu[subMenuM.membersID[subMenuM.linesToScreen[subMenuM.pointerPos]]].min))
            {
                uint16_t val = eeprom_read_word(&editableMenu[subMenuM.membersID[subMenuM.linesToScreen[subMenuM.pointerPos]]]);
                val--;
                eeprom_write_word((uint16_t *)&editableMenu[subMenuM.membersID[subMenuM.linesToScreen[subMenuM.pointerPos]]], val);
            }
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
    WDT(WDTO_250MS, 6);
    // uint8_t lineHight = 16;
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
    WDT(WDTO_8S, 10);
    oldTimer = 0;
#ifdef v220V
    dimmer = HEATER_MAX;
    attachInterrupt(INT_NUM, isr, RISING);
    Timer1.enableISR();
#else
    analogWrite(DIMMER_PIN, HEATER_MIN); // TODO: проверить
#endif
    piii(100);
    state = DRY;

    iDryer.data.flag = 1;
    iDryer.data.flagTimeCounter = 0;
    iDryer.data.flagScreenUpdate = 1;

    // iDryer.data.setTemp = eeprom_read_word(&menuVal[DEF_DRYING_TEMPERATURE]);
    // iDryer.data.setTime = eeprom_read_word(&menuVal[DEF_DRYING_TIME]);

    pid.SetTunings(double(iDryer.data.Kp), double(iDryer.data.Ki), double(iDryer.data.Kd), P_ON_E);
    pid.SetSampleTime(iDryer.data.sampleTime);

    iDryer.data.startTime = millis();
#ifdef WITH_BLACKJACK_AND_HOOKERS
    Servo1.test();
    // Servo2.test();
#endif
    WDT_DISABLE();
}

void storageStart()
{
    WDT(WDTO_8S, 11);
    oldTimer = 0;
#ifdef v220V
    dimmer = HEATER_MAX;
    attachInterrupt(INT_NUM, isr, RISING);
    Timer1.enableISR();
#else
    analogWrite(DIMMER_PIN, HEATER_MIN); // TODO: проверить
#endif
    piii(100);
    state = STORAGE;

    iDryer.data.flag = 1;
    iDryer.data.flagTimeCounter = 0;
    iDryer.data.flagScreenUpdate = 1;
    iDryer.data.setTemp = eeprom_read_word(&menuVal[DEF_STORAGE_TEMPERATURE]); // TODO
    iDryer.data.setHumidity = eeprom_read_word(&menuVal[DEF_STORAGE_HUMIDITY]);

    pid.SetTunings(double(iDryer.data.Kp), double(iDryer.data.Ki), double(iDryer.data.Kd), P_ON_E);
    pid.SetSampleTime(iDryer.data.sampleTime);

#ifdef WITH_BLACKJACK_AND_HOOKERS
    Servo1.test();
    // Servo2.test();
#endif
    WDT_DISABLE();
}

void autoPidM()
{
    WDT(WDTO_250MS, 12);
#ifdef v220V
    attachInterrupt(INT_NUM, isr, RISING);
    Timer1.enableISR();
#else
    // digitalWrite(DIMMER_PIN, 0);
#endif

#ifdef DEBUG
    wdt_reset();
    wdt_disable();
#endif
    piii(100);

    state = AUTOPID;
    iDryer.data.flag = 1;
    iDryer.data.flagTimeCounter = 0;
    iDryer.data.setTemp = eeprom_read_word(&menuVal[DEF_AVTOPID_TEMPERATURE]);
    analogWrite(FAN, map(iDryer.data.setFan, 0, 100, 0, 255));
    // pid.SetTunings(double(iDryer.data.Kp), double(iDryer.data.Ki), double(iDryer.data.Kd), P_ON_E);
    // pid.SetTunings(1, 0.1, 2, P_ON_E);
    WDT_DISABLE();
}

void updateIDyerData()
{
    WDT(WDTO_250MS, 4);
    iDryer.data.setFan = eeprom_read_word(&menuVal[DEF_SETTINGS_BLOWING]);
    iDryer.data.Kp = (double)eeprom_read_word(&menuVal[DEF_PID_KP]) / 100.00;
    iDryer.data.Ki = (double)eeprom_read_word(&menuVal[DEF_PID_KI]) / 100.00;
    iDryer.data.Kd = (double)eeprom_read_word(&menuVal[DEF_PID_KD]) / 100.00;
    iDryer.data.sampleTime = eeprom_read_word(&menuVal[DEF_AVTOPID_TIME_MS]);
    iDryer.data.deltaT = eeprom_read_word(&menuVal[DEF_SETTINGS_DELTA]);
    iDryer.data.setHumidity = eeprom_read_word(&menuVal[DEF_STORAGE_HUMIDITY]);
    iDryer.data.setFan = eeprom_read_word(&menuVal[DEF_SETTINGS_BLOWING]);
    pid.SetTunings(double(iDryer.data.Kp), double(iDryer.data.Ki), double(iDryer.data.Kd), P_ON_E);
    pid.SetSampleTime(iDryer.data.sampleTime);
#ifdef WITH_BLACKJACK_AND_HOOKERS
    Servo1.set(eeprom_read_word(&menuVal[DEF_SERVO1_CLOSED]), eeprom_read_word(&menuVal[DEF_SERVO1_OPEN]), eeprom_read_word(&menuVal[DEF_SERVO1_CORNER]));
    // Servo2.set(eeprom_read_word(&menuVal[DEF_SERVO2_CLOSED]), eeprom_read_word(&menuVal[DEF_SERVO2_OPEN]), eeprom_read_word(&menuVal[DEF_SERVO2_CORNER]));
#endif
    WDT_DISABLE();
}

void saveAll()
{
    updateIDyerData();
    oled.firstPage();
    do
    {
        // TODO: Добавить ОК или СОХРАНЕНО
        oled.drawUTF8((128 - oled.getUTF8Width(printMenuItem(&serviceTxt[DEF_T_AYRA]))) / 2, lineHight * 3, printMenuItem(&serviceTxt[DEF_T_AYRA]));
    } while (oled.nextPage());
    delay(500);
    subMenuM.pointerUpdate = 1;
#ifdef WITH_BLACKJACK_AND_HOOKERS

#else

#endif
}

// WDTO_15MS
// WDTO_8S
void WDT(uint16_t time, uint8_t current_function_uuid)
{
    globalErrorFuncUUID = current_function_uuid;
#ifdef WITH_BLACKJACK_AND_HOOKERS
    wdt_reset();
    wdt_disable();
#ifdef DEBUG
#else
    wdt_enable(time); // TODO после перезагрузки по ресету сделать уведомление при старте, что был ресет
    WDTCSR |= (1 << WDIE);
#endif
#endif
}

ISR(WDT_vect)
{
    WDT_ERROR |= (1UL << globalErrorFuncUUID);
#ifdef KASYAK_FINDER
    Serial.print("WDT ERROR: ");
    Serial.println(WDT_ERROR);
#endif
    eeprom_write_dword((uint32_t *)&ERROR_CODE, WDT_ERROR);
    // WDTCSR |= (1<<WDIF); //!! reboot
}

void WDT_DISABLE()
{
#ifdef WITH_BLACKJACK_AND_HOOKERS
    wdt_reset();
    wdt_disable();
#endif
}

bool setError(uint8_t errorCode)
{
    WDT_ERROR |= (1UL << errorCode);
    // WDT_ERROR = 0b10000000000100000000001000000000;
    eeprom_write_dword(&ERROR_CODE, WDT_ERROR);
    return true;
}

uint32_t readError()
{
    uint32_t wdt_error = eeprom_read_dword(&ERROR_CODE);
    // printError(wdt_error);
    return printError(wdt_error);
}

uint32_t printError(uint32_t error)
{
    uint32_t errorCounter = 0;
    for (uint8_t i = 0; i < 32; i++)
    {
        // uint32_t errorTmp = error & (1UL << i); // ((error) >> (i)) & 0x01;
        if (error & (1UL << i))
        {
            errorTmpArray[i] = 1;
            errorCounter++;
        }
    }
    return errorCounter;
}

void piii(uint16_t time_ms)
{
    WDT(WDTO_8S, 11);
    time_ms > 4 ? 4 : time_ms;
    // analogWrite(BUZZER_PIN, BUZZER_PWM);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(time_ms);
    // analogWrite(BUZZER_PIN, 0);
    digitalWrite(BUZZER_PIN, LOW);
    WDT_DISABLE();
}

void heaterON(uint16_t Output, uint16_t &dimmer)
{
    WDT(WDTO_500MS, 2);
#ifdef v220V
    dimmer = uint16_t(HEATER_MAX + HEATER_MIN - Output);
#else
    analogWrite(DIMMER_PIN, Output);
#endif
    WDT_DISABLE();
}

void heaterOFF()
{
    WDT(WDTO_500MS, 1);
#ifdef v220V
    detachInterrupt(INT_NUM);
    // Timer1.disableISR();
    dimmer = 0;
    Timer1.stop();
    dimmer = HEATER_MAX;
    digitalWrite(DIMMER_PIN, 0);
#else
    digitalWrite(DIMMER_PIN, 0);
#endif
    WDT_DISABLE();
}


void pwm_test()
{
    uint16_t frequency = 0;
    for(uint8_t i = 0; i < 11; i++)
    {
        switch (i)
        {
        case 0: //30
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
        uint8_t pwm = 255;
        uint8_t k = 100;
        while (k > 0)
        {
            analogWrite(FAN, 0);
            delay(2000);
            analogWrite(FAN, 255 / 100 * k);
            oled.clear();
            oled.firstPage();
            do
            {
                oled.setFont(u8g2_font);
                sprintf(serviceString, "ЧАСТОТА:  %6hu", frequency);
                oled.drawUTF8((128 - oled.getUTF8Width(serviceString)) / 2, lineHight * 2, serviceString);
                sprintf(serviceString, "%4s %6hu", printMenuItem(&menuTxt[DEF_SETTINGS_BLOWING]), k);
                oled.drawUTF8((128 - oled.getUTF8Width(serviceString)) / 2, lineHight * 3, serviceString);
            } while (oled.nextPage());
            delay(3000);
            k -= 10;
        }
    }
}