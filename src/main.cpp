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
#include "menu.h"
#include "font.h"



#if REV == 0
#define ERROR
#elif REV == 1
#define v220V
#elif REV == 2
#define v24V
#endif

// #define DEBUG

#define NTC_PIN 0
// Димер
#ifdef v220V
#define ZERO_PIN 2 // пин детектора нуля
#define INT_NUM 0  // соответствующий управляющему пину номер прерывания
#endif
#define DIMMER_PIN 5 // управляющий пин симистора или пин ШИМ

//!* 11,6,5,3
#define BUZZER_PIN 3
#define FAN 11
#define SERVO_1 7
#define SERVO_2 8
#define EXT_PWM 6

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

#define TMP_MIN 1
#define TMP_MAX 120

#define ADC_MIN 50
#define ADC_MAX 1020

uint8_t lineHight = 16;
uint16_t dimmer = 230; // переменная диммера
#ifdef v220V
uint16_t lastDim;
#endif

uint16_t ERROR_TO_EPROM = 0b0000000000000000;

uint8_t autoPidAttemptCounter = 0;

stateS state = MENU;
uint8_t ERROR_COUNTER = 0;

#ifdef DEBUG
uint8_t testPWM = 0;
uint16_t testTIMER_COUNT = 0;
uint16_t testTIMER_STATE = 0;
unsigned long oldTime1 = 0;
unsigned long oldTime2 = 0;
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
unsigned long screenTime = 0;
char seviceString[25];

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

PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, DIRECT);

struct Data
{
    uint8_t ntcTemp = 0;
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
            // Serial.println("!=");
            return true;
        }
        else
        {
            // Serial.println("==");
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
void updateIDyerData();
void encoderSate(struct control *control);
void controlsHandler(const menuS constMenu[], uint16_t editableMenu[], const ptrFunc functionMenu[], struct control *encoder, struct subMenu *subMenu);
void screen(struct subMenu *subMenu);
void submenuHandler(const menuS constMenu[], uint8_t menuSize, struct subMenu *subMenu);

class iDryer
{
public:
    Data data;
    Data oldData;
    bool getData()
    {
        data.ntcTemp = ((uint8_t)ntc.analog2temp() + data.ntcTemp) / 2;
        data.bmeTemp = (bme.readTemperature() + data.bmeTemp) / 2;
        data.bmeHumidity = (bme.readHumidity() + data.bmeHumidity) / 2;
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

        if (data.bmeTemp > data.setTemp - data.deltaT / 2 )
        {
            data.flagTimeUpdate = true;
        }

        if (data.ntcTemp < TMP_MIN ||
            data.ntcTemp > TMP_MAX ||
            data.bmeTemp < TMP_MIN ||
            data.bmeTemp > TMP_MAX)
        {
            setError(13);
            return false;
        }
        return true;
    }

} iDryer;

#ifdef v220V
void isr()
{
#ifdef DEBUG
    testTIMER_COUNT++;
    testTIMER_STATE = 1;
    oldTime2 = millis() - oldTime1;
    oldTime1 = millis();
    isrFlag = 1;
#endif
    digitalWrite(DIMMER_PIN, 0); // выключаем симистор
    // если значение изменилось, устанавливаем новый период
    // если нет, то просто перезапускаем со старым
    if (lastDim != dimmer)
    {
        Timer1.setPeriod(lastDim = dimmer);
    }
    else
    {
        Timer1.restart();
    }
}

ISR(TIMER1_A)
{
    digitalWrite(DIMMER_PIN, 1); // включаем симистор
    Timer1.stop();               // останавливаем таймер
#ifdef DEBUG
    testTIMER_STATE = 0;
#endif
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
        enc.tick();
        isr_1 = isr_2 = isr_3 = 0;
    }
    oldPorta = PINC;
}

char *printMenuItem(const char *const *text) // печать строки из prm
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
void dispalyPrint(struct subMenu *subMenu)
{
    oled.firstPage();
    do
    {
        oled.setFont(u8g2_font);
        uint8_t maxPos = SCREEN_LINES - MENU_HEADER < subMenu->membersQuantity ? SCREEN_LINES - MENU_HEADER : subMenu->membersQuantity;
        // Serial.println("maxPos: " + String(maxPos));
        oled.drawUTF8((128 - oled.getUTF8Width(printMenuItem(&menuPGM[subMenuM.parentID].text))) / 2, 16, printMenuItem(&menuPGM[subMenuM.parentID].text));
        oled.drawLine(0, lineHight + 2, 128, lineHight + 2);
        for (uint8_t i = 0; i < maxPos; i++)
        {
            oled.drawUTF8(0, (i + 2) * lineHight, printMenuItem(&menuPGM[subMenuM.membersID[subMenuM.linesToScreen[i]]].text));
            // controls.ok = 0;
            if (pgm_read_word(&menuPGM[subMenuM.membersID[subMenuM.linesToScreen[i]]].min) ||
                pgm_read_word(&menuPGM[subMenuM.membersID[subMenuM.linesToScreen[i]]].max) ||
                menuVal[subMenuM.membersID[subMenuM.linesToScreen[i]]])
            {
                char val[4];
                sprintf(val, "%3d", menuVal[subMenuM.membersID[subMenuM.linesToScreen[i]]]);
                oled.drawUTF8(100, (i + 2) * lineHight, val);
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

void dispalyPrintMode()
{
    ERROR_TO_EPROM = 0;
    oled.firstPage();
    do
    {
        oled.setFont(u8g2_font);

        uint8_t text = 0;
        if (state == DRY)
        {
            text = 0;
        }
        if (state == STORAGE)
        {
            text = 1;
        }

        char val[4];

        oled.drawUTF8((128 - oled.getUTF8Width(printMenuItem(&txt[text]))) / 2, lineHight, printMenuItem(&txt[text]));
        sprintf(val, "%3d", iDryer.data.setTime);
        oled.drawUTF8(100, lineHight, val);
        oled.drawButtonUTF8(0, lineHight - 1, U8G2_BTN_INV, 128, 1, 1, "");
        oled.drawLine(0, lineHight + 2, 128, lineHight + 2);

        for (uint8_t i = 2; i < 5; i++)
        {
            oled.drawUTF8(0, (i)*lineHight, printMenuItem(&txt[i]));
            text = i;
            uint8_t data = 0;
            switch (i)
            {
            case 2: // Воздух
                data = iDryer.data.bmeTemp;
                break;
            case 3: // Нагреватель
                data = iDryer.data.ntcTemp;
                break;
            case 4: // Влажность
                data = iDryer.data.bmeHumidity;
                break;
            default:
                break;
            }
            sprintf(val, "%3d", data);
            oled.drawUTF8(100, (i)*lineHight, val);
        }
        Serial.print(">- ");
    } while (oled.nextPage());
    Serial.println("->");
    iDryer.data.flagScreenUpdate = 0;
}

void setup()
{
    Serial.begin(115200);
    menuSize = sizeof(menuPGM) / sizeof(menuPGM[0]);

    uint8_t errorCode = readError();
    if (errorCode)
    {
        ERROR_TO_EPROM = 0;
        EEPROM.put(sizeof(menuVal) / sizeof(menuVal[0]), ERROR_TO_EPROM);
    }

#ifdef DEBUG
    Serial.println("settingsSize: " + String(settingsSize));
    Serial.println("menuSize: " + String(menuSize));
#endif

    uint16_t test;
    EEPROM.get(0, test);
    if (test != 123)
    {
#if K_PROPRTIONAL != 0
        menuVal[24] = K_PROPRTIONAL;
        menuVal[25] = K_INTEGRAL;
        menuVal[26] = K_DERIVATIVE;
        menuVal[28] = K_SAMPLE_TIME;
#endif
        EEPROM.put(0, menuVal);
    }

    delay(10);
    EEPROM.get(0, menuVal);

    updateIDyerData();

    myPID.SetMode(AUTOMATIC); //
    myPID.SetControllerDirection(DIRECT);
    myPID.SetOutputLimits(0, HEATER_MAX);
    myPID.SetTunings(double(iDryer.data.Kp), double(iDryer.data.Ki), double(iDryer.data.Kd), P_ON_M);
    myPID.SetSampleTime(iDryer.data.sampleTime);

// Диммер
#ifdef v220V
    pinMode(ZERO_PIN, INPUT_PULLUP);
#endif

    pinMode(DT, INPUT_PULLUP);
    pinMode(CLK, INPUT_PULLUP);
    pinMode(encBut, INPUT_PULLUP);
    pinMode(DIMMER_PIN, OUTPUT);
    pinMode(SERVO_1, OUTPUT);

    oldPorta = PINC;
    PCMSK1 |= (1 << PCINT9);
    PCMSK1 |= (1 << PCINT10);
    PCMSK1 |= (1 << PCINT11);
    // Serial.println(PCMSK1,HEX);
    PCMask = PCMSK1;
    PCICR |= (1 << PCIE1);

    TCCR2B = 0b00000010;
    pinMode(BUZZER_PIN, OUTPUT);
    // analogWrite(BUZZER_PIN, 0);
    // digitalWrite(BUZZER_PIN, 0);
    pinMode(FAN, OUTPUT);

#ifdef v220V
    attachInterrupt(INT_NUM, isr, FALLING); // для самодельной схемы ставь FALLING
    Timer1.enableISR();                     // Timer2.enableISR();
#endif

    oled.begin();
    oled.setFlipMode(SCREEN_FLIP);
    // oled.setContrast(1);
    oled.enableUTF8Print();
    oled.setFont(u8g2_font); // u8g2_font   u8g2_font

    oled.firstPage();
    do
    {
        oled.drawUTF8((128 - oled.getUTF8Width(printMenuItem(&serviceTxt[1]))) / 2, lineHight, printMenuItem(&serviceTxt[1]));
        // oled.drawUTF8((128 - oled.getUTF8Width((class __FlashStringHelper *)f0)) / 2,  lineHight, (class __FlashStringHelper *)f0);
        oled.drawUTF8((128 - oled.getUTF8Width(printMenuItem(&serviceTxt[2]))) / 2, lineHight * 2, printMenuItem(&serviceTxt[2]));
        oled.drawUTF8((128 - oled.getUTF8Width(printMenuItem(&serviceTxt[3]))) / 2, lineHight * 3, printMenuItem(&serviceTxt[3]));
        if (errorCode)
        {
            sprintf(seviceString, "%8s %d", printMenuItem(&txt[5]), errorCode);
            oled.drawUTF8((128 - oled.getUTF8Width(seviceString)) / 2, lineHight * 4, seviceString);
            oled.drawButtonUTF8(0, lineHight * 4, U8G2_BTN_INV, 128, 1, 1, "");
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
}

void loop()
{
    delay(500);
        servoPulse(SERVO_1, 0);
    delay(500);
        servoPulse(SERVO_1, 90);
    delay(500);
        servoPulse(SERVO_1, 180);

    // for (int i = 0; i <= 50; i++)
    // {
    //     servoPulse(SERVO_1, 90);
    // }

    // for (int i = 0; i <= 50; i++)
    // {
    //     servoPulse(SERVO_1, 0);
    // }


    int tmpTemp = analogRead(NTC_PIN);
    // if (tmpTemp <= ADC_MIN || tmpTemp >= ADC_MAX || !iDryer.getData())
    if (tmpTemp <= ADC_MIN || tmpTemp >= ADC_MAX)
    {
#ifdef DEBUG
        Serial.println("NTC ERROR");
#endif
        setError(15);
        state = NTC_ERROR;
    }

    if (!digitalRead(encBut) && (state == DRY || state == STORAGE))
    {
        state = MENU;
        digitalWrite(DIMMER_PIN, 0);
        dimmer = 0;
        piii(500);
        subMenuM.levelUpdate = UP;
        subMenuM.parentID = 0;
        subMenuM.pointerPos = 0;
        subMenuM.pointerUpdate = 1;
        delay(700);
        enc.resetStates();
    }

#ifdef DEBUG
    // Serial.println("switch (state): " + String(state));
#endif

    switch (state)
    {
    case NTC_ERROR:
        WDT(WDTO_4S);
        dimmer = 0;
        digitalWrite(DIMMER_PIN, 0);
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
            oled.drawUTF8((128 - oled.getUTF8Width(printMenuItem(&txt[5]))) / 2, lineHight, printMenuItem(&txt[5]));
            oled.drawUTF8((128 - oled.getUTF8Width(printMenuItem(&txt[6]))) / 2, lineHight * 2, printMenuItem(&txt[6]));
            oled.drawUTF8((128 - oled.getUTF8Width(printMenuItem(&txt[7]))) / 2, lineHight * 3, printMenuItem(&txt[7]));
            oled.drawUTF8((128 - oled.getUTF8Width(printMenuItem(&txt[8]))) / 2, lineHight * 4, printMenuItem(&txt[8]));
            oled.drawButtonUTF8(0, 1 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
            oled.drawButtonUTF8(0, 2 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
            oled.drawButtonUTF8(0, 3 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
            oled.drawButtonUTF8(0, 4 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
        } while (oled.nextPage());

        while (1)
        {
            dimmer = 0;
            digitalWrite(DIMMER_PIN, 0);

#ifdef v220V
            detachInterrupt(INT_NUM);
#endif

            analogWrite(FAN, 255);
            // analogWrite(BUZZER_PIN, BUZZER_PWM);
        }
        break;
    case OFF:
        WDT(WDTO_1S);
        /* code */
        break;
    case ON:
        WDT(WDTO_1S);
        // #ifdef DEBUG
        //         Serial.println("-----------> state: ON");
        // #endif
        break;
    case MENU:
        WDT(WDTO_2S);
        // #ifdef DEBUG
        //         Serial.println("-----------> state: MENU");
        // #endif
        analogWrite(FAN, 0);        //!! Что-то придумать!!!
        analogWrite(DIMMER_PIN, 0); //!! Что-то придумать!!!
        enc.tick();
        encoderSate(&controls);
        controlsHandler(menuPGM, menuVal, menuFunc, &controls, &subMenuM);
        if (subMenuM.levelUpdate)
        {
            submenuHandler(menuPGM, menuSize, &subMenuM);
            screen(&subMenuM); //!! добавить subMenuM.pointerUpdate  в submenuHandler
            dispalyPrint(&subMenuM);
        }
        if (subMenuM.pointerUpdate)
        {
            screen(&subMenuM);
            dispalyPrint(&subMenuM);
        }
        break;
    case DRY:
        WDT(WDTO_500MS);
        if (iDryer.getData())
        {
            if (iDryer.data.flagScreenUpdate)
                dispalyPrintMode();
        }
        else
        {
            ERROR_COUNTER++;
            if (ERROR_COUNTER > MAX_ERROR)
            {
                setError(14);
                state = NTC_ERROR;
            }
        }

        if (iDryer.data.bmeTemp < iDryer.data.setTemp - 2)
        {
            myPID.SetTunings(double(iDryer.data.Kp), double(iDryer.data.Ki), double(iDryer.data.Kd), P_ON_E);
        }
        else
        {
            myPID.SetTunings(double(iDryer.data.Kp), double(iDryer.data.Ki), double(iDryer.data.Kd), P_ON_M);
        }

        if (iDryer.data.ntcTemp >= iDryer.data.setTemp) // TODO:  првоерить что это целевая температура iDryer.temperature
        {
            if (millis() - oldTime > 1000)
            {
                oldTime = millis();
                Setpoint = iDryer.data.setTemp + iDryer.data.ntcTemp - iDryer.data.bmeTemp;
                if (Setpoint > iDryer.data.setTemp + iDryer.data.deltaT)
                    Setpoint = iDryer.data.setTemp + iDryer.data.deltaT;
            }
        }
        else
        {
            Setpoint = iDryer.data.setTemp + iDryer.data.deltaT / 2;
        }

        Input = double(iDryer.data.ntcTemp);
        myPID.Compute();

#ifdef v220V
#ifdef DEBUG
        Serial.println(regulator.getResult());
#endif
        dimmer = map(Output, 0, 255, 500, 9300);
#endif

#ifdef v24V
#ifdef DEBUG
#endif
        // Serial.println("setpoint: " + String(regulator.setpoint) + "\tntcTemp: " + String(iDryer.data.ntcTemp) +  "PWM: " + String(regulator.getResult()));
        analogWrite(DIMMER_PIN, Output);
#endif
        // TODO подобрать условие включения часов


        if (millis() - oldTimer >= 60000 && iDryer.data.flagTimeUpdate)
        {
            
            oldTimer = millis();
            iDryer.data.setTime--;
        }

        if (iDryer.data.setTime == 0)
        {
            oldTimer = 0;
            state = STORAGE; // автоматический переход в режим хранения
        }
        break;
    case STORAGE:
        WDT(WDTO_500MS);
        if (iDryer.getData())
        {
            if (iDryer.data.flagScreenUpdate)
                dispalyPrintMode();
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

        // if (iDryer.data.ntcTemp < Setpoint && iDryer.data.setTemp - Setpoint > 5)
        if (iDryer.data.bmeTemp < iDryer.data.setTemp - 2)
        {
            myPID.SetTunings(double(iDryer.data.Kp), double(iDryer.data.Ki), double(iDryer.data.Kd), P_ON_E);
        }
        else
        {
            myPID.SetTunings(double(iDryer.data.Kp), double(iDryer.data.Ki), double(iDryer.data.Kd), P_ON_M);
        }

        if (iDryer.data.ntcTemp >= iDryer.data.setTemp) // TODO:  првоерить что это целевая температура iDryer.temperature
        {
            if (millis() - oldTime > 1000)
            {
                oldTime = millis();
                Setpoint = iDryer.data.setTemp + iDryer.data.ntcTemp - iDryer.data.bmeTemp;
                if (Setpoint > iDryer.data.setTemp + iDryer.data.deltaT)
                    Setpoint = iDryer.data.setTemp + iDryer.data.deltaT;
            }
        }
        else
        {
            Setpoint = iDryer.data.setTemp + iDryer.data.deltaT / 2;
        }

        if ((iDryer.data.bmeHumidity > iDryer.data.setHumidity && iDryer.data.flag) ||
            (iDryer.data.bmeHumidity > iDryer.data.setHumidity + 2 && !iDryer.data.flag)) // TODO 5 вынести куданить
        {
            iDryer.data.flag = 1;
            Input = double(iDryer.data.ntcTemp);
            myPID.Compute();

#ifdef v220V
            dimmer = map(Output, 0, 255, 500, 9300);
#endif

#ifdef v24V
            analogWrite(DIMMER_PIN, Output);
#endif
        }
        else
        {
            iDryer.data.flag = 0;
            Input = double(iDryer.data.ntcTemp);
            Setpoint = 0;
            myPID.Compute();
#ifdef v220V
            dimmer = 0;
#endif

#ifdef v24V
            analogWrite(DIMMER_PIN, 0);
#endif
            // Serial.print("\tHEATER NAH!: ");
        }

        break;
    case AUTOPID:
        WDT(WDTO_1S);
        analogWrite(FAN, iDryer.data.setFan);

        PIDAutotuner tuner = PIDAutotuner();
        tuner.setTargetInputValue(iDryer.data.setTemp);
        tuner.setLoopInterval(uint32_t(iDryer.data.sampleTime) * 1000);
        tuner.setTuningCycles(AUTOPID_ATTEMPT);
        tuner.setOutputRange(0, HEATER_MAX);
        // ZNModeNoOvershoot - Defaults,   ZNModeBasicPID
        tuner.setZNMode(PIDAutotuner::ZNModeBasicPID);
        tuner.startTuningLoop(micros());

        oled.clear();
        unsigned long microseconds;
        // oldTime = millis();
        // oldTimer = micros();
        WDT(WDTO_1S);
        while (!tuner.isFinished())
        {
            WDT(WDTO_4S);
            microseconds = micros();
            iDryer.getData();

#ifdef v220V
            // dimmer = map(tuner.getOutput(), 0, 255, 500, 9300);
            dimmer = map(uint8_t(tuner.tunePID(double(iDryer.data.ntcTemp), microseconds)), 0, 255, 500, 9300);
#endif
#ifdef v24V
            analogWrite(DIMMER_PIN, uint8_t(tuner.tunePID(double(iDryer.data.ntcTemp), microseconds)));
#endif

            screenTime = millis();
            uint32_t new_loop = uint32_t(iDryer.data.sampleTime) * 1000;
            oled.firstPage();
            do
            {
                oled.setFont(u8g2_font);

                sprintf(seviceString, "ТСТ-%d/%d  %dС", tuner.getCycle(), AUTOPID_ATTEMPT, (uint8_t)ntc.analog2temp());
                oled.drawUTF8((128 - oled.getUTF8Width(seviceString)) / 2, lineHight * 1, seviceString);

                sprintf(seviceString, "КП %5d", (uint16_t)(round(double(tuner.getKp()))));
                oled.drawUTF8((128 - oled.getUTF8Width(seviceString)) / 2, lineHight * 2, seviceString);

                sprintf(seviceString, "КИ %5d", (uint16_t)(round(double(tuner.getKi()))));
                oled.drawUTF8((128 - oled.getUTF8Width(seviceString)) / 2, lineHight * 3, seviceString);

                sprintf(seviceString, "КД %5d", (uint16_t)(round(double(tuner.getKd()))));
                oled.drawUTF8((128 - oled.getUTF8Width(seviceString)) / 2, lineHight * 4, seviceString);

            } while (oled.nextPage());
            delay(iDryer.data.sampleTime - uint16_t(millis() - screenTime)); // while (micros() - microseconds < new_loop)
        }
        WDT(WDTO_1S);
        dimmer = 0;
        delay(20);
        analogWrite(DIMMER_PIN, 0);

        // Сохраняем
        menuVal[24] = (uint16_t)(round(double(tuner.getKp())));
        menuVal[25] = (uint16_t)(round(double(tuner.getKi())));
        menuVal[26] = (uint16_t)(round(double(tuner.getKd())));
        menuVal[32] = iDryer.data.ntcTemp - iDryer.data.bmeTemp;
        saveAll();

        oled.clear();
        do
        {
            oled.setFont(u8g2_font);
            oled.drawUTF8((128 - oled.getUTF8Width(printMenuItem(&menuPGM[23].text))) / 2, lineHight * 2, printMenuItem(&menuPGM[23].text));
            oled.drawUTF8((128 - oled.getUTF8Width(printMenuItem(&serviceTxt[5]))) / 2, lineHight * 2, printMenuItem(&serviceTxt[5]));
            // sprintf(seviceString, "СОХРАНЕНЕНО");
            // oled.drawUTF8((128 - oled.getUTF8Width(seviceString)) / 2, lineHight * 3, seviceString);
            // oled.drawUTF8(20, lineHight * 3, *seviceString);
            // oled.drawButtonUTF8(0, 1 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
            // oled.drawButtonUTF8(0, 2 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
            // oled.drawButtonUTF8(0, 3 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
            // oled.drawButtonUTF8(0, 4 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
        } while (oled.nextPage());

        analogWrite(FAN, 250);
        delay(1000);

        oled.clear();
        // TODO Сделать обратный отсчет секунд
        do
        {
            oled.setFont(u8g2_font);
            sprintf(seviceString, "ОХЛАЖДЕНИЕ");
            oled.drawUTF8((128 - oled.getUTF8Width(printMenuItem(&serviceTxt[4]))) / 2, lineHight * 2, printMenuItem(&serviceTxt[4]));

        } while (oled.nextPage());
        wdt_disable();
        delay(60000);

        iDryer.data.flagScreenUpdate = 1;
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
        // Serial.println("\t\t  enc.isClick()");
    }
    if (enc.isHolded())
    {
        control->hold = 1;
        // Serial.print("\t\t  enc.isHolded()");
    }
    if (enc.isRelease())
    {
        enc.resetStates();
        // Serial.print(" enc.isRelease() control->hold = 1;");
    }
    if (enc.isLeft())
    {
        control->left = 1;
        // Serial.print("enc.isLeft()");
    }
    if (enc.isRight())
    {
        control->right = 1;
        // Serial.print("enc.isRight()");
    }
}

void controlsHandler(const menuS constMenu[], uint16_t editableMenu[], const ptrFunc functionMenu[], struct control *encoder, struct subMenu *subMenu)
{
    if (encoder->ok)
    {
        encoder->ok = false;
        // Serial.print("encoder->ok");
        if (!pgm_read_word(&constMenu[subMenu->membersID[subMenu->position]].min) &&
            !pgm_read_word(&constMenu[subMenu->membersID[subMenu->position]].max) &&
            //! pgm_read_byte(&constMenu[subMenu->membersID[subMenu->position]].action)) // меню˚
            !functionMenu[subMenu->membersID[subMenu->position]]) // меню
        {
            subMenu->level + 1 > subMenu->levelMax ? subMenu->level : subMenu->level++;
            subMenu->levelUpdate = DOWN;
        }
        else
        {
            if (functionMenu[subMenu->membersID[subMenu->position]]) // подменю с функцией
            {
                // funcNum = subMenu->membersID[subMenu->linesToScreen[subMenu->pointerPos]];
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

    if (encoder->right)
    {
        encoder->right = false;
        if (!subMenu->changeVal)
        {
            subMenu->position == subMenu->max ? subMenu->position = 0 : subMenu->position++;
        }
        else
        {
            editableMenu[subMenuM.membersID[subMenuM.linesToScreen[subMenuM.pointerPos]]] ==
                    pgm_read_word(&constMenu[subMenuM.membersID[subMenuM.linesToScreen[subMenuM.pointerPos]]].max)
                ? pgm_read_word(&constMenu[subMenuM.membersID[subMenuM.linesToScreen[subMenuM.pointerPos]]].max)
                : editableMenu[subMenuM.membersID[subMenuM.linesToScreen[subMenuM.pointerPos]]]++;
        }
        subMenu->pointerUpdate = true;
    }

    if (encoder->left)
    {
        encoder->left = false;
        if (!subMenu->changeVal)
        {
            subMenu->position == subMenu->min ? subMenu->position = subMenu->max : subMenu->position--;
        }
        else
        {
            editableMenu[subMenuM.membersID[subMenuM.linesToScreen[subMenuM.pointerPos]]] ==
                    pgm_read_word(&constMenu[subMenuM.membersID[subMenuM.linesToScreen[subMenuM.pointerPos]]].min)
                ? pgm_read_word(&constMenu[subMenuM.membersID[subMenuM.linesToScreen[subMenuM.pointerPos]]].min)
                : editableMenu[subMenuM.membersID[subMenuM.linesToScreen[subMenuM.pointerPos]]]--;
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
#ifdef DEBUG
        Serial.print("subMenu->parentID: ");
        Serial.println(subMenu->parentID);
#endif
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
        // Serial.println("i: " + String(i));
        if (pgm_read_byte(&constMenu[i].parentID) == subMenu->parentID && pgm_read_byte(&constMenu[i].id) != 0)
        {
            subMenu->membersID[iCounter] = i;
            // Serial.print("pgm_read_byte(&constMenu[i].parentID): " + String(pgm_read_byte(&constMenu[i].parentID)));
            // Serial.print("    pgm_read_byte(&constMenu[i].id): " + String(pgm_read_byte(&constMenu[i].id)));
            // Serial.print("  subMenu->members: " + String(printMenuItem(&constMenu[subMenu->membersID[iCounter]].text)));
            // Serial.println("  menuVal: " + String(menuVal[subMenu->membersID[iCounter]]));
            delay(20);
            if (i == posTmp)
            {
                // Serial.println(String(i) + " : " + String(printMenuItem(&constMenu[i].text)) + "\tsubmenuHandler subMenu->position" + String(subMenu->position));
                subMenu->position = iCounter;
                subMenu->pointerUpdate = 1;
            }
            iCounter++;
        }
        // Serial.println("");
    }
    subMenu->membersQuantity = iCounter;
    // Serial.print("subMenu->membersQuantity: ");
    // Serial.println(subMenu->membersQuantity);
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
    piii(100);
    saveAll();
    state = DRY;
    EEPROM.put(0, menuVal);
    iDryer.data.flag = 1;
    iDryer.data.flagTimeUpdate = 0;
    iDryer.data.flagScreenUpdate = 1;
    iDryer.data.setTemp = menuVal[subMenuM.parentID + 1];
    iDryer.data.setTime = menuVal[subMenuM.parentID + 2];
    analogWrite(FAN, map(iDryer.data.setFan, 0, 100, 0, 255));
    iDryer.data.startTime = millis();
}

void storageStart()
{
    piii(100);
    saveAll();
    state = STORAGE;
    EEPROM.put(0, menuVal);
    iDryer.data.flag = 1;
    iDryer.data.flagTimeUpdate = 0;
    iDryer.data.flagScreenUpdate = 1;
    iDryer.data.setTemp = menuVal[subMenuM.parentID + 1];
    iDryer.data.setHumidity = menuVal[subMenuM.parentID + 2];
    analogWrite(FAN, map(iDryer.data.setFan, 0, 100, 0, 255));
}

void autoPidM()
{
    piii(100);
    saveAll();
    state = AUTOPID;
    iDryer.data.flag = 1;
    iDryer.data.flagTimeUpdate = 0;
    iDryer.data.setTemp = menuVal[29];
    analogWrite(FAN, map(iDryer.data.setFan, 0, 100, 0, 255));
    myPID.SetTunings(double(iDryer.data.Kp), double(iDryer.data.Ki), double(iDryer.data.Kd), P_ON_E);
}

void updateIDyerData()
{
    iDryer.data.setFan = menuVal[31];
    iDryer.data.Kp = menuVal[24];
    iDryer.data.Ki = menuVal[25];
    iDryer.data.Kd = menuVal[26];
    iDryer.data.sampleTime = menuVal[28];
    iDryer.data.deltaT = menuVal[32];
    iDryer.data.setHumidity = menuVal[7];
    myPID.SetSampleTime(iDryer.data.sampleTime);
}

void saveAll()
{
    updateIDyerData();
    EEPROM.put(0, menuVal);
}

// WDTO_15MS
// WDTO_8S
void WDT(uint16_t time)
{
#ifdef BOOTLOADER_WDT
    wdt_reset();
    wdt_disable();
    wdt_enable(time); // TODO после перезагрузки по ресету сделать уведомление при старте, что был ресет
#endif
}

// ERROR CODE
//  15 - adc read error
//  14 - ERROR_COUNTER error
//  13 - getDate() error
bool setError(uint16_t errorCode)
{
    ERROR_TO_EPROM |= (1 << errorCode);
    EEPROM.put(sizeof(menuVal) / sizeof(menuVal[0]), ERROR_TO_EPROM);
    // printError(ERROR_TO_EPROM);
    // EEPROM.get(sizeof(menuVal)/sizeof(menuVal[0]), ERROR_TO_EPROM);
    // printError(ERROR_TO_EPROM);
    return true;
}

uint8_t readError()
{
    EEPROM.get(sizeof(menuVal) / sizeof(menuVal[0]), ERROR_TO_EPROM);
    return printError(ERROR_TO_EPROM);
}

uint8_t printError(uint16_t error)
{
    for (uint8_t i = 0; i < 16; i++)
    {
        Serial.print(i);
        Serial.print(": ");
        Serial.println(((error) >> (i)) & 0x01);
    }
    for (uint8_t i = 0; i < 16; i++)
    {
        if ((((error) >> (i)) & 0x01) == 1)
            return i;
    }
    return 0;
}

void piii(uint16_t time_ms)
{
    analogWrite(BUZZER_PIN, BUZZER_PWM);
    delay(time_ms);
    analogWrite(BUZZER_PIN, 0);
}

void servoPulse(int pin, int angle)
{
    piii(SERVO_CUCKOO);
	int pulsewidth = map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE); 
    for (int pulseCounter = 0; pulseCounter <= 50; pulseCounter++){
    unsigned long startTime = micros();
    digitalWrite(pin,HIGH);
    while(micros() - startTime < pulsewidth){}
    digitalWrite(pin,LOW);
    while(micros() - startTime < SERVO_REIOD_MS * 1000){}
    }
}