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
// #include <PIDAutotuner.h>
#include <thermistor.h>
#include <Configuration.h>
#include "font.h"
#include "menu.h"
#include "def.h"

uint16_t ERROR_CODE EEMEM; //EEMEM = 0b0000000000000000;

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
#define FAN 6
#define EXT_PWM 11

#define SERVO_1_PIN 7
#define SERVO_2_PIN 8

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
volatile byte flagISR = 0, intsFound = 0;
byte isr_1 = 0;
byte isr_2 = 0;
byte isr_3 = 0;

uint8_t funcNum = 0;

uint8_t menuSize = 0;
uint8_t settingsSize = 0;
uint8_t buzzerAlarm = 0;
unsigned long oldTime = 0;
unsigned long oldTimer = 0;
unsigned long servoOldTime1 = 0;
unsigned long servoOldTime2 = 0;
uint32_t timerOn = 0;
uint32_t timerOff = 0;
uint8_t servoState1 = CLOSED;
uint8_t servoState2 = CLOSED;
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

double mainSetpoint, Setpoint, Input, Output;

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
    uint8_t setTemp = 0;
    uint8_t setHumidity = 0;
    uint16_t setTime = 0;
    uint8_t flag = 0;
    uint8_t flagScreenUpdate = 0;
    uint8_t flagTimeUpdate = 0;
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

void servoPulse(int pin, int angle);
void piii(uint16_t time_ms);
uint8_t printError(uint16_t error);
uint8_t readError();
bool setError(uint16_t errorCode);
void WDT(uint16_t time);
void WDT_DISABLE();
void updateIDyerData();
void encoderSate(struct control *control);
void controlsHandler(const menuS constMenu[], uint16_t editableMenu[], const ptrFunc functionMenu[], struct control *encoder, struct subMenu *subMenu);
void screen(struct subMenu *subMenu);
void submenuHandler(const menuS constMenu[], uint8_t menuSize, struct subMenu *subMenu);
void heaterOFF();
void heaterON(uint16_t Output, uint16_t& dimmer);


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

        if (data.bmeTemp > data.setTemp && !data.flagTimeUpdate)
        {
            data.flagTimeUpdate = true;
        }

        if (data.ntcTemp < TMP_MIN ||
            data.ntcTemp > TMP_MAX + 10 ||
            data.bmeTemp < TMP_MIN ||
            data.bmeTemp > TMP_MAX + 10)
        {
//!! setError(13);
#ifdef DEBUG
#else
        return false;
#endif
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
    }
    void set(uint16_t _closedTime, uint16_t _openTime, uint16_t _angle)
    {
        closedTime = _closedTime;
        openTime = _openTime;
        angle = _angle;
    }
    void test(uint16_t _closedTime, uint16_t _openTime, uint16_t _angle)
    {
        // closedTime = _closedTime;
        // openTime = _openTime;
        // angle = _angle;
        // if (state == OPEN)
        // {
        //     servoPulse(pin, 90);
        //     state = CLOSED;
        //     oldTime = millis();
        // }
        // if (state == CLOSED)
        // {
        //     servoPulse(pin, 90 + angle);
        //     state = OPEN;
        //     oldTime = millis();
        // }
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
        if(state == CLOSED)
        {
            if ( oldTime < millis())
            {
                servoPulse(pin, 90 + angle);
                state = OPEN;
                oldTime = millis() + openTime * 1000 * 60;
            }
        }
        if(state == OPEN)
        {
            if ( oldTime < millis())
            {
                servoPulse(pin, 90);
                state = CLOSED;
                oldTime = millis() + closedTime * 1000 * 60;
            }
        }
    }

    void servoPulse(int pin, int angle)
    {
        WDT(WDTO_4S);
        // piii(SERVO_CUCKOO);
        int pulsewidth = map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
        for (int pulseCounter = 0; pulseCounter <= 50; pulseCounter++)
        {
            unsigned long startTime = micros();
            digitalWrite(pin, HIGH);
            while (micros() - startTime < pulsewidth)
            {
            }
            digitalWrite(pin, LOW);
            while (micros() - startTime < SERVO_PERIOD_MS * 1000)
            {
            }
        }
    }
};
servo Servo1(SERVO_1_PIN, eeprom_read_word(&menuVal[DEF_SERVO_SERVO1 + 1]), eeprom_read_word(&menuVal[DEF_SERVO_SERVO1 + 2]), eeprom_read_word(&menuVal[DEF_SERVO_SERVO1 + 3]));
servo Servo2(SERVO_2_PIN, eeprom_read_word(&menuVal[DEF_SERVO_SERVO1 + 1]), eeprom_read_word(&menuVal[DEF_SERVO_SERVO1 + 2]), eeprom_read_word(&menuVal[DEF_SERVO_SERVO1 + 3]));
#endif

#ifdef v220V
void isr()
{
#ifdef DEBUG
testTIMER_COUNT ++;
#endif
    if (state == DRY || state == STORAGE || state == AUTOPID)
    {
        if (lastDim != dimmer)
        {
            Timer2.setPeriod(lastDim = dimmer);
        }
        else
        {
            Timer2.restart();
        }    

        Timer2.setPeriod(dimmer);
    }
}

ISR(TIMER2_A)
{
    digitalWrite(DIMMER_PIN, 1);
    digitalWrite(DIMMER_PIN, 0);
    Timer2.stop();
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
    WDT(WDTO_4S);
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
}

void displayPrintMode()
{
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
}

void setup()
{
// Serial.begin(9600);
#ifdef v220V
    dimmer = HEATER_MAX;
    
    pinMode(ZERO_PIN, INPUT_PULLUP);
    pinMode(DIMMER_PIN, OUTPUT);
    // attachInterrupt(INT_NUM, isr, RISING);
    // Timer2.enableISR();
#endif

    menuSize = sizeof(menuTxt) / sizeof(menuTxt[0]);
    uint8_t errorCode = readError();

    //Serial.println("error:" + String(errorCode));
    if (errorCode)
    {
        // Serial.println("eeprom_write_word:");
        eeprom_write_word((uint16_t*)&ERROR_CODE, 0b0000000000000000);
    }

    uint16_t test;

    test = eeprom_read_word(&menuVal[0]);

    // Serial.println("chek:" + String(test));
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
    // else
    // {
    //     Serial.println("EEPROM READY");
    // }

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

    // for(uint8_t i = 0; i < 43; i++)
    // {
    //     Serial.print(i);
    //     Serial.print(": ");
    //     Serial.println(eeprom_read_word(&menuVal[i]));
    // }

    updateIDyerData();

    pid.SetMode(AUTOMATIC); //
    #ifdef v24V
    pid.SetControllerDirection(DIRECT); // REVERSE
    pid.SetOutputLimits(HEATER_MIN, HEATER_MAX);
    #endif
    #ifdef v220V
    pid.SetControllerDirection(DIRECT); // REVERSE
    pid.SetOutputLimits(HEATER_MIN, HEATER_MAX);
    #endif
    pid.SetTunings(double(iDryer.data.Kp), double(iDryer.data.Ki), double(iDryer.data.Kd), P_ON_E);
    pid.SetSampleTime(iDryer.data.sampleTime);

    // oldPorta = PINC;
    PCMSK1 |= (1 << PCINT9);
    PCMSK1 |= (1 << PCINT10);
    PCMSK1 |= (1 << PCINT11);
    // Serial.println(PCMSK1,HEX);
    PCMask = PCMSK1;
    PCICR |= (1 << PCIE1);

    TCCR1A = 0b00000001;
    TCCR1B = 0b00001010;

    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(FAN, OUTPUT);
    pinMode(SERVO_1_PIN, OUTPUT);
    pinMode(SERVO_2_PIN, OUTPUT);

    oled.begin();
    oled.setFlipMode(SCREEN_FLIP);
    // oled.setContrast(1);
    oled.enableUTF8Print();
    oled.setFont(u8g2_font); // u8g2_font

    oled.firstPage();
    do
    {
        oled.drawUTF8((128 - oled.getUTF8Width(printMenuItem(&serviceTxt[DEF_T_AYRA]))) / 2, lineHight, printMenuItem(&serviceTxt[DEF_T_AYRA]));
        oled.drawUTF8((128 - oled.getUTF8Width(printMenuItem(&serviceTxt[DEF_T_VERSION]))) / 2, lineHight * 2, printMenuItem(&serviceTxt[DEF_T_VERSION]));
        oled.drawUTF8((128 - oled.getUTF8Width(printMenuItem(&serviceTxt[DEF_T_VER]))) / 2, lineHight * 3, printMenuItem(&serviceTxt[DEF_T_VER]));
        if (errorCode)
        {
            {
                sprintf(serviceString, "%8s %hu", printMenuItem(&serviceTxt[DEF_T_ERROR]), errorCode);
                oled.drawUTF8((128 - oled.getUTF8Width(serviceString)) / 2, lineHight * 4, serviceString);
                oled.drawButtonUTF8(0, lineHight * 4, U8G2_BTN_INV, 128, 1, 1, "");
            }
        }
    } while (oled.nextPage());
    if (errorCode)
    {
        for (uint8_t i = 0; i < 16; i++)
        {
            analogWrite(BUZZER_PIN, BUZZER_PWM);
            delay(500);
            analogWrite(BUZZER_PIN, 0);
            delay(500);
        }
    }

    delay(1000);

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
    // servoPulse(SERVO_1_PIN, 180);
    // servoPulse(SERVO_1_PIN, 90);
    // servoPulse(SERVO_2_PIN, 180);
    // servoPulse(SERVO_2_PIN, 90);
    // servoState1 == CLOSED;
    // servoState2 == CLOSED;

Servo1.test();
Servo2.test();
#endif
}

void loop()
{
// dimmer >= 9500 ? dimmer = 6000 : dimmer+=10;

    enc.tick();
    uint16_t tmpTemp = analogRead(NTC_PIN);
    if (tmpTemp <= ADC_MIN || tmpTemp >= ADC_MAX)
    {
        setError(15);
        state = NTC_ERROR;
    }

    if (enc.isHold() && (state == DRY || state == STORAGE))
    {
        WDT_DISABLE();

        state = MENU;
        // digitalWrite(DIMMER_PIN, 0);
        // Timer2.stop();
        // dimmer = HEATER_MIN;
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
        WDT_DISABLE();
        heaterOFF();
#ifdef v220V
        detachInterrupt(INT_NUM);
#endif
        analogWrite(FAN, 255);
        analogWrite(BUZZER_PIN, BUZZER_PWM);
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

        analogWrite(FAN, 255);
        analogWrite(BUZZER_PIN, BUZZER_PWM);
        while (1)
        {
            heaterOFF();
        }
        break;
    case OFF:
        WDT(WDTO_1S);
        break;
    case ON:
        WDT(WDTO_1S);
        break;
    case MENU:
        WDT(WDTO_2S);
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

        // analogWrite(DIMMER_PIN, 0); //!! Что-то придумать!!!
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
        WDT(WDTO_2S);
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
                setError(14);
                state = NTC_ERROR;
                // Serial.println("851");
            }
        }

        iDryer.data.flagTimeUpdate ? analogWrite(FAN, map(iDryer.data.setFan, 0, 100, 0, 255)) : analogWrite(FAN, 255);

        // pid.SetTunings(double(iDryer.data.Kp), double(iDryer.data.Ki), double(iDryer.data.Kd), P_ON_E);
        
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

// #ifdef v220V
//         // dimmer = (uint16_t)Output;
//         dimmer = HEATER_MAX + HEATER_MIN - Output;
// #endif
// #ifdef v24V
// #ifdef DEBUG
// #endif
//         analogWrite(DIMMER_PIN, Output);
// #endif

        heaterON(Output, dimmer);

        // TODO подобрать условие включения часов
        if (millis() - oldTimer >= 60000 && iDryer.data.flagTimeUpdate)
        {
            oldTimer = millis();
            iDryer.data.setTime--;
        }
        if (iDryer.data.setTime == 0)
        {
            subMenuM.parentID = 5; // ID пункта хранение
            delay(100);
            heaterOFF();
            storageStart();
        }
#ifdef WITH_BLACKJACK_AND_HOOKERS
        Servo1.check();
        Servo2.check();
#endif
        break;
    case STORAGE:
        WDT(WDTO_500MS);
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
                setError(15);
                state = NTC_ERROR;
            }
        }

        iDryer.data.flagTimeUpdate ? analogWrite(FAN, map(iDryer.data.setFan, 0, 100, 0, 255)) : analogWrite(FAN, 255);
        
        // pid.SetTunings(double(iDryer.data.Kp), double(iDryer.data.Ki), double(iDryer.data.Kd), P_ON_E);

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

        if ((iDryer.data.bmeHumidity > iDryer.data.setHumidity && iDryer.data.flag) ||
            (iDryer.data.bmeHumidity > iDryer.data.setHumidity + 2 && !iDryer.data.flag))
        {
            iDryer.data.flag = 1;
            Input = double(iDryer.data.ntcTemp);
            pid.Compute();

// #ifdef v220V
//             // dimmer = map(Output, 0, 255, HEATER_MIN, HEATER_MAX);
//             dimmer = Output;
// #endif
// #ifdef v24V
//             analogWrite(DIMMER_PIN, Output);
// #endif
        heaterON(Output, dimmer);
        }
        else
        {
            iDryer.data.flag = 0;
            Input = double(iDryer.data.ntcTemp);
            // Setpoint = 0;
            // pid.Compute();
            heaterON(Output = 0, dimmer);
// #ifdef v220V
//             dimmer = HEATER_MIN;
// #endif
// #ifdef v24V
//             analogWrite(DIMMER_PIN, 0);
// #endif
            // Serial.print("\tHEATER NAH!: ");
        }

#ifdef WITH_BLACKJACK_AND_HOOKERS
        Servo1.check();
        Servo2.check();
#endif

        break;
    case AUTOPID:
        WDT(WDTO_4S);
        PIDAutotuner tuner = PIDAutotuner();
        tuner.setTargetInputValue((float)iDryer.data.setTemp);
        tuner.setLoopInterval(long(iDryer.data.sampleTime) * 1000);
        tuner.setTuningCycles(AUTOPID_ATTEMPT);
        tuner.setOutputRange(HEATER_MIN, HEATER_MAX);
        tuner.setZNMode(PIDAutotuner::ZNModeBasicPID);         // ZNModeNoOvershoot - Defaults,   ZNModeBasicPID
        tuner.startTuningLoop(micros());

        oled.clear();
        do
        {
            oled.setFont(u8g2_font);
            oled.drawUTF8((128 - oled.getUTF8Width(printMenuItem(&menuTxt[DEF_PID_AVTOPID]))) / 2, lineHight * 3, printMenuItem(&menuTxt[DEF_PID_AVTOPID]));
            oled.drawButtonUTF8(0, 1 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
            oled.drawButtonUTF8(0, 2 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
            oled.drawButtonUTF8(0, 3 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
            oled.drawButtonUTF8(0, 4 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
        } while (oled.nextPage());
        
        unsigned long microseconds;
        WDT(WDTO_2S);

        analogWrite(FAN, 255);
        WDT_DISABLE();
        while (!tuner.isFinished())
        {
            microseconds = micros();
            iDryer.getData();
// #ifdef v220V
//             // dimmer = (uint16_t)(tuner.tunePID(double(iDryer.data.ntcTemp), microseconds));
//             dimmer = HEATER_MAX + HEATER_MIN - (uint16_t)(tuner.tunePID(double(iDryer.data.ntcTemp), microseconds));
// #endif
// #ifdef v24V
//             analogWrite(DIMMER_PIN, uint8_t(tuner.tunePID(double(iDryer.data.ntcTemp), microseconds)));
// #endif
            
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

            while (micros() - microseconds < (unsigned long)iDryer.data.sampleTime * 1000){}
        }
        WDT(WDTO_2S);
        // #ifdef v220V
        // dimmer = HEATER_MAX;
        // #else
        // dimmer = HEATER_MIN;
        // #endif
        // analogWrite(DIMMER_PIN, 0);
        heaterOFF();
        analogWrite(FAN, 250);
        
        eeprom_update_word(&menuVal[DEF_PID_KP], (uint16_t)(abs(tuner.getKp()) * 100));
        eeprom_update_word(&menuVal[DEF_PID_KI], (uint16_t)(abs(tuner.getKi()) * 100));
        eeprom_update_word(&menuVal[DEF_PID_KD], (uint16_t)(abs(tuner.getKd()) * 100));
        eeprom_update_word(&menuVal[DEF_SETTINGS_DELTA], iDryer.data.ntcTemp - iDryer.data.bmeTemp);
        saveAll();

        oled.clear();
        do
        {
            oled.setFont(u8g2_font);
            oled.drawUTF8((128 - oled.getUTF8Width(printMenuItem(&menuTxt[DEF_SETTINGS_PID]))) / 2, lineHight * 2, printMenuItem(&menuTxt[DEF_SETTINGS_PID]));
            oled.drawUTF8((128 - oled.getUTF8Width(printMenuItem(&serviceTxt[5]))) / 2, lineHight * 3, printMenuItem(&serviceTxt[5]));
        } while (oled.nextPage());
        WDT_DISABLE();

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

        WDT(WDTO_1S);

        oled.clear();

        wdt_disable();

        subMenuM.levelUpdate = DOWN;
        subMenuM.pointerUpdate = 1;
        subMenuM.parentID = 0;
        subMenuM.position = 0;
        memset(subMenuM.membersID, 0, sizeof(subMenuM.membersID) / sizeof(subMenuM.membersID[0]));

        // iDryer.data.flagScreenUpdate = 1;
        state = MENU;
        WDT(WDTO_2S);
        break;
    }
}

// TODO запилить нормальный энкодер
void encoderSate(struct control *control)
{
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
}

void controlsHandler(const menuS constMenu[], uint16_t editableMenu[], const ptrFunc functionMenu[], struct control *encoder, struct subMenu *subMenu)
{
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
    if (encoder->right) // Timer2.enableISR();
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
    if (encoder->left) // Timer2.enableISR();
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
}

void submenuHandler(const menuS constMenu[], uint8_t menuSize, struct subMenu *subMenu)
{
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
}

void screen(struct subMenu *subMenu)
{
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
}

void dryStart()
{
    oldTimer = 0;
    attachInterrupt(INT_NUM, isr, RISING);
    Timer2.enableISR();
    piii(100);
    state = DRY;

    iDryer.data.flag = 1;
    iDryer.data.flagTimeUpdate = 0;
    iDryer.data.flagScreenUpdate = 1;

    iDryer.data.setTemp = eeprom_read_word(&menuVal[subMenuM.parentID + 1]);
    iDryer.data.setTime = eeprom_read_word(&menuVal[subMenuM.parentID + 2]);

    iDryer.data.startTime = millis();
#ifdef WITH_BLACKJACK_AND_HOOKERS
Servo1.test();
Servo2.test();
#endif
}

void storageStart()
{
    oldTimer = 0;
    attachInterrupt(INT_NUM, isr, RISING);
    Timer2.enableISR();
    piii(100);
    state = STORAGE;

    iDryer.data.flag = 1;
    iDryer.data.flagTimeUpdate = 0;
    iDryer.data.flagScreenUpdate = 1;
    iDryer.data.setTemp = eeprom_read_word(&menuVal[subMenuM.parentID + 1]); // TODO
    iDryer.data.setHumidity = eeprom_read_word(&menuVal[subMenuM.parentID + 2]);

#ifdef WITH_BLACKJACK_AND_HOOKERS
    Servo1.test();
    Servo2.test();
#endif
}

void autoPidM()
{
    attachInterrupt(INT_NUM, isr, RISING);
    Timer2.enableISR();
    WDT(WDTO_250MS);
#ifdef DEBUG
    wdt_reset();
    wdt_disable();
#endif
    piii(100);
    
    state = AUTOPID;
    iDryer.data.flag = 1;
    iDryer.data.flagTimeUpdate = 0;
    iDryer.data.setTemp = eeprom_read_word(&menuVal[DEF_AVTOPID_TEMPERATURE]);
    analogWrite(FAN, map(iDryer.data.setFan, 0, 100, 0, 255));
    pid.SetTunings(double(iDryer.data.Kp), double(iDryer.data.Ki), double(iDryer.data.Kd), P_ON_E);
}

void updateIDyerData()
{
    iDryer.data.setFan = eeprom_read_word(&menuVal[DEF_SETTINGS_BLOWING]);
    iDryer.data.Kp = (double)eeprom_read_word(&menuVal[DEF_PID_KP]) / 100.00;
    iDryer.data.Ki = (double)eeprom_read_word(&menuVal[DEF_PID_KI]) / 100.00;
    iDryer.data.Kd = (double)eeprom_read_word(&menuVal[DEF_PID_KD]) / 100.00;
    iDryer.data.sampleTime = eeprom_read_word(&menuVal[DEF_AVTOPID_TIME_MS]);
    iDryer.data.deltaT = eeprom_read_word(&menuVal[DEF_SETTINGS_DELTA]);
    iDryer.data.setHumidity = eeprom_read_word(&menuVal[DEF_STORAGE_HUMIDITY]);
    pid.SetTunings(double(iDryer.data.Kp), double(iDryer.data.Ki), double(iDryer.data.Kd), P_ON_E);
    pid.SetSampleTime(iDryer.data.sampleTime);
}

void saveAll()
{
    updateIDyerData();
#ifdef WITH_BLACKJACK_AND_HOOKERS

#else

#endif
}

// WDTO_15MS
// WDTO_8S
void WDT(uint16_t time)
{
#ifdef WITH_BLACKJACK_AND_HOOKERS
    wdt_reset();
    wdt_disable();
#ifdef DEBUG
#else
    wdt_enable(time); // TODO после перезагрузки по ресету сделать уведомление при старте, что был ресет
#endif
#endif
}

void WDT_DISABLE()
{
#ifdef WITH_BLACKJACK_AND_HOOKERS
    wdt_reset();
    wdt_disable();
#endif
}

// ERROR CODE
//  15 - adc read error
//  14 - ERROR_COUNTER error
//  13 - getDate() error
bool setError(uint16_t errorCode)
{
    ERROR_CODE |= (1 << errorCode);
    eeprom_write_word((uint16_t *)&ERROR_CODE, ERROR_CODE);
    printError(ERROR_CODE);
    return true;
}

uint8_t readError()
{
    uint16_t errorCode = eeprom_read_word((uint16_t *)&ERROR_CODE);
    return printError(errorCode);
}

uint8_t printError(uint16_t error)
{
    for (uint8_t i = 0; i < 16; i++)
    {
        uint8_t errorTmp = ((error) >> (i)) & 0x01;
        if (errorTmp != 0)
        {
            eeprom_write_word((uint16_t *)&ERROR_CODE, 0b0000000000000000);
            return i;
        }
    }

//!!! 
// TODO 
    // for (uint8_t i = 0; i < 16; i++)
    // {
    //     Serial.print(i);
    //     Serial.print(":");
    //     Serial.print(((error) >> (i)) & 0x01);
    //     Serial.print("  | ");
    // }
    // Serial.println("");

    return 0;
}

void piii(uint16_t time_ms)
{
    analogWrite(BUZZER_PIN, BUZZER_PWM);
    delay(time_ms);
    analogWrite(BUZZER_PIN, 0);
}

void heaterON(uint16_t Output, uint16_t& dimmer)
{
    #ifdef v220V
    dimmer = uint16_t(HEATER_MAX + HEATER_MIN - Output);
    #else
    analogWrite(DIMMER_PIN, Output);    
    #endif
}

void heaterOFF()
{
    #ifdef v220V
    detachInterrupt(INT_NUM);
    Timer2.stop();
    dimmer = HEATER_MAX;
    digitalWrite(DIMMER_PIN, 0);
    #else
    digitalWrite(DIMMER_PIN, 0);  
    #endif
}