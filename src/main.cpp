#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <EEPROM.h>
#include <GyverTimers.h> // библиотека таймера
#include <GyverEncoder.h>
#include <GyverBME280.h>
// #include "GyverPID.h"
// #include "PIDtuner2.h"
// #include "PIDtuner.h"
#include <PID_v1.h>
#include <pidautotuner.h> //!! https://github.com/jackw01/arduino-pid-autotuner
#include <thermistor.h>
#include "menu.h"
#include "font.h"

//!! Выбери свой термистор TEMP_SENSOR_0 из списка в файле Configuration.h
// TODO перетащить дефайны етрмисторов сюда
#define NTC_PIN 0

// #define DEBUG
//!! Раскомментируй для своей версии платы
// #define v220V
#define v24V
// Укажи мощность нагревателя YUOR_HEATER_POWER
#define HEATER_POWER   80
// Переворот экрана 0 - нормально, 1 - перевернуто
#define SCREEN_FLIP 1 


//Точность в процентах (диапазон 95-100). 
//Если автопид крутится бесконечно, снизь точность
#define ACCURANCY   100 //TODO перенести в меню
#define MIN_POWER   40
#define MAX_POWER   500
#define HEATER_PWM map(HEATER_POWER, MIN_POWER, MAX_POWER, 210, 100)

// Димер
#ifdef v220V
#define ZERO_PIN 2 // пин детектора нуля
#define INT_NUM 0  // соответствующий управляющему пину номер прерывания
#endif
#define DIMMER_PIN 5 // управляющий пин симистора или пин ШИМ

//!* 11,6,5,3
#define BUZZER_PIN 3
#define FAN 11

#define HESTERESIS 5

#define SCREEN_LINES 4
#define MENU_HEADER 1

// #define OFF 0
// #define ON 1
// #define MENU 2
// #define DRY 3
// #define STORAGE 4
// #define AUTOPID 5
// #define NTC_ERROR 6

typedef enum stateS
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
#define AUTOPID_ATTEMPT 7

#define TMP_MIN 10
#define TMP_MAX 120

#define ADC_MIN 200
#define ADC_MAX 1000

uint8_t lineHight = 16;
uint16_t dimmer = 230; // переменная диммера
#ifdef v220V
uint16_t lastDim;
#endif

uint8_t autoPidAttemptCounter = 0;

// uint8_t state = MENU;
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

enum levelV
{
    NOTHING,
    DOWN,
    UP,
};

// void testFuc_1()
// {
//     Serial.println("УРРРРА\nЗА\nРА\nБО\nТА\nЛА\n");
// }

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

struct Settings
{
    uint8_t state = 0;
    double pidKp = 0;
    double pidKi = 0;
    double pidKd = 0;
    uint16_t pidDt = 0;
};

control controls;
subMenu subMenuM;
Settings settings;

thermistor ntc(NTC_PIN, 0);
GyverBME280 bme;

// В случае необходимости поменяй экран
//  U8G2_SH1106_128X64_NONAME_1_HW_I2C oled(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
U8G2_SSD1306_128X64_NONAME_1_HW_I2C oled(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

Encoder enc(CLK, DT, encBut, TYPE1);

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
};

class iDryer
{
public:
    uint8_t temperature = 0;
    Data data;
    bool getData()
    {
        data.ntcTemp = (uint8_t)ntc.analog2temp();
        data.bmeTemp = bme.readTemperature();
        data.bmeHumidity = bme.readHumidity();

        // if (data.bmeTemp < data.setTemp - HESTERESIS)
        //     temperature = data.setTemp + 10;
        // if (data.bmeTemp > data.setTemp - HESTERESIS / 2)
        //     temperature = data.setTemp + 5;
        // if (data.bmeTemp > data.setTemp - HESTERESIS && data.bmeTemp < data.setTemp + HESTERESIS)
        //     temperature = data.setTemp;
        // if (data.bmeTemp >= data.setTemp - HESTERESIS / 2)
        //     temperature = data.setTemp;

        // if (data.bmeTemp <= data.setTemp)
        // {
        // temperature = (data.ntcTemp + data.bmeTemp) / 2;
        // }
        // else
        // {
        //     temperature = data.setTemp;
        // }
        temperature = data.setTemp;

        
        if (data.ntcTemp < TMP_MIN ||
            data.ntcTemp > TMP_MAX ||
            data.bmeTemp < TMP_MIN ||
            data.bmeTemp > TMP_MAX)
        {
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
    static char buffer[20];
    uint16_t ptr = pgm_read_word(text);
    uint8_t i = 0;

    do
    {
        buffer[i] = (char)(pgm_read_byte(ptr++));
    } while (i++ != 20);

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
}

void dispalyPrintMode()
{
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
        for (uint8_t i = 2; i < 6; i++)
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
    } while (oled.nextPage());
}

void encoderSate(struct control *control);
void controlsHandler(const menuS constMenu[], uint16_t editableMenu[], const ptrFunc functionMenu[], struct control *encoder, struct subMenu *subMenu);
void screen(struct subMenu *subMenu);
void submenuHandler(const menuS constMenu[], uint8_t menuSize, struct subMenu *subMenu);

void setup()
{

    Serial.begin(115200);

    settingsSize = sizeof(settings);
    menuSize = sizeof(menuPGM) / sizeof(menuPGM[0]);

#ifdef DEBUG
    Serial.println("settingsSize: " + String(settingsSize));
    Serial.println("menuSize: " + String(menuSize));
#endif

    EEPROM.get(0, settings);

    if (settings.state != 123)
    {
        settings.state = 123;
        settings.pidKp = 148.70;
        settings.pidKi = 22.86;
        settings.pidKd = 241.85;
        settings.pidDt = 500;

        EEPROM.put(0, settings);
        delay(10);
    }

    myPID.SetMode(AUTOMATIC); //
    myPID.SetControllerDirection(DIRECT);
    myPID.SetTunings(double(settings.pidKp), double(settings.pidKi), double(settings.pidKd), P_ON_M);
    myPID.SetSampleTime(settings.pidDt);


    Serial.println(settings.state);
    Serial.println(settings.pidKp);
    Serial.println(settings.pidKi);
    Serial.println(settings.pidKd);
    Serial.println(settings.pidDt);

    uint16_t test;
    EEPROM.get(settingsSize, test);

    if (test != 123)
    {
        EEPROM.put(settingsSize, menuVal);
    }
    delay(10);
    EEPROM.get(settingsSize, menuVal);

// Диммер
#ifdef v220V
    pinMode(ZERO_PIN, INPUT_PULLUP);
#endif

    pinMode(DIMMER_PIN, OUTPUT);

    pinMode(DT, INPUT_PULLUP);
    pinMode(CLK, INPUT_PULLUP);
    pinMode(encBut, INPUT_PULLUP);

    oldPorta = PINC;
    PCMSK1 |= (1 << PCINT9);
    PCMSK1 |= (1 << PCINT10);
    PCMSK1 |= (1 << PCINT11);
    // Serial.println(PCMSK1,HEX);
    PCMask = PCMSK1;
    PCICR |= (1 << PCIE1);

    pinMode(BUZZER_PIN, OUTPUT);
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
        oled.drawUTF8(0, 16, "");
        oled.drawUTF8(32, 32, "АЙДРАЙ");
        oled.drawUTF8(32, 48, "ВЕР 001");
        oled.drawUTF8(0, 64, "");
        oled.drawButtonUTF8(0, 32 - 1, U8G2_BTN_INV, 128, 1, 1, "");

    } while (oled.nextPage());
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

}

void loop()
{
#ifdef DEBUG
    // analogWrite(FAN, 230);
    // analogWrite(DIMMER_PIN, 230);
    //analogWrite(BUZZER_PIN, 490 / 255 * 1);
    // Serial.println("state: " + String(state));
    
    // if(millis() % 5 == 0) Serial.println("testPWM: " + String(testPWM) + "\ttestTIMER_STATE: " + String(testTIMER_STATE) + "\ttestTIMER_COUNT: " + String(testTIMER_COUNT) + "\tdimmer: " + String(dimmer));

    // if(isrFlag)
    // {
    // // Serial.println("\toldTime2: " + String(oldTime2));
    // // Serial.println("testPWM: " + String(testPWM++) + "\tdimmer: " + String(dimmer) + "\ttestTIMER_STATE: " + String(testTIMER_STATE) + "\ttestTIMER_COUNT: " + String(testTIMER_COUNT) + "\toldTime2: " + String(oldTime2));
    // // Serial.println("\tdimmer: " + String(dimmer) + "\ttestTIMER_STATE: " + String(testTIMER_STATE) + "\ttestTIMER_COUNT: " + String(testTIMER_COUNT) + "\toldTime2: " + String(oldTime2));
    // Serial.println("\tdimmer: " + String(dimmer));
    // dimmer < 9300 ? dimmer++ : dimmer = 230;
    // isrFlag = 0;
    // // dimmer = map(testPWM++, 0, 255, 500, 9300);
    // }

    // if (millis() >= oldTime + 10)
    // {
    //     PIND = PIND | 0b00001000;
    //     oldTime = millis();
    //     if(testTIMER_STATE % 10) testPWM++;
    //     // dimmer = map(testPWM, 0, 255, 500, 9300);
    //     dimmer = 0;
    // }
    // dimmer = 0;
#endif

    int tmpTemp = analogRead(NTC_PIN);
    if (tmpTemp <= ADC_MIN || tmpTemp >= ADC_MAX)
    {
#ifdef DEBUG
        Serial.println("NTC ERROR");
#endif
        state = NTC_ERROR;
    }

    if (!digitalRead(encBut) && (state == DRY || state == STORAGE))
    {
        state = MENU;
        digitalWrite(DIMMER_PIN, 0);
        // tone(BUZZER_PIN, 500, 100);
        enc.resetStates();
        subMenuM.levelUpdate = UP;
        subMenuM.pointerUpdate = 1;
        subMenuM.parentID = 0;
        delay(100);
    }

#ifdef DEBUG
    // Serial.println("switch (state): " + String(state));
#endif

    switch (state)
    {
    case NTC_ERROR:
        dimmer = 0;
        digitalWrite(DIMMER_PIN, 0);
#ifdef v220V
        detachInterrupt(INT_NUM);
#endif
        analogWrite(FAN, 255);
        analogWrite(BUZZER_PIN, 150);
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
            analogWrite(BUZZER_PIN, buzzerAlarm++);
        }
        break;
    case OFF:
        /* code */
        break;
    case ON:
// #ifdef DEBUG
//         Serial.println("-----------> state: ON");
// #endif
        break;
    case MENU:

// #ifdef DEBUG
//         Serial.println("-----------> state: MENU");
// #endif
        analogWrite(FAN, 0); //!! Что-то придумать!!!
        analogWrite(DIMMER_PIN, 0); //!! Что-то придумать!!!
        enc.tick();
        encoderSate(&controls);
        controlsHandler(menuPGM, menuVal, menuFunc, &controls, &subMenuM);
        if (subMenuM.levelUpdate)
        {
            submenuHandler(menuPGM, menuSize, &subMenuM);
            screen(&subMenuM); //!! добавить subMenuM.pointerUpdate  в submenuHandler
        }
        if (subMenuM.pointerUpdate)
        {
            screen(&subMenuM);
        }
        dispalyPrint(&subMenuM);
        break;
    case DRY:
        if (iDryer.getData())
        {
            dispalyPrintMode();
        }
        else
        {
            ERROR_COUNTER++;
            if (ERROR_COUNTER > MAX_ERROR)
                state = NTC_ERROR;
        }

        if (iDryer.data.ntcTemp < Setpoint && iDryer.temperature - Setpoint > 5)
        {
            myPID.SetMode(AUTOMATIC); //
            myPID.SetControllerDirection(DIRECT);
            myPID.SetSampleTime(settings.pidDt);
            myPID.SetTunings(double(settings.pidKp), double(settings.pidKi), double(settings.pidKd), P_ON_E);
        }
        else
        {
            myPID.SetMode(AUTOMATIC); //
            myPID.SetControllerDirection(DIRECT);
            myPID.SetSampleTime(settings.pidDt);
            myPID.SetTunings(double(settings.pidKp), double(settings.pidKi), double(settings.pidKd),P_ON_M);
        }

        if (iDryer.data.ntcTemp >= iDryer.temperature) //TODO:  првоерить что это целевая температура iDryer.temperature
        {
            if (millis() - oldTime > 1000)
            {
            oldTime = millis();
            Setpoint = iDryer.temperature + iDryer.data.ntcTemp - iDryer.data.bmeTemp;
            if (Setpoint > iDryer.temperature + 10) Setpoint = iDryer.temperature + 10;
            }
        }
        else
        {
        Setpoint = iDryer.temperature + 2;
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
        //Serial.println("ntcTemp: " + String(uint8_t(iDryer.data.ntcTemp)) +  "\tbmeTemp: " + String(uint8_t(iDryer.data.bmeTemp)) + "\tPWM: " + String(uint8_t(regulator.getResult()))); //
        if (millis() - oldTime >= 1000 * 60) //!!НАПИСАТЬ ИМЕНА
        {
            oldTime = millis();
            // Serial.println("Прошла минута");
            iDryer.data.setTime--;
        }

        if (iDryer.data.setTime == 0)
        {
            oldTime = 0;
            state = STORAGE; // автоматический переход в режим хранения
        }
        
        break;
    case STORAGE:
        // Serial.println("STORAGE");
        if (iDryer.getData())
        {
            dispalyPrintMode();
        }
        else
        {
            ERROR_COUNTER++;
            if (ERROR_COUNTER > MAX_ERROR)
                state = NTC_ERROR;
        }


        if (iDryer.data.ntcTemp < Setpoint && iDryer.temperature - Setpoint > 5)
        {
            myPID.SetMode(AUTOMATIC); //
            myPID.SetControllerDirection(DIRECT);
            myPID.SetSampleTime(settings.pidDt);
            myPID.SetTunings(double(settings.pidKp), double(settings.pidKi), double(settings.pidKd), P_ON_E);
        }
        else
        {
            myPID.SetMode(AUTOMATIC); //
            myPID.SetControllerDirection(DIRECT);
            myPID.SetSampleTime(settings.pidDt);
            myPID.SetTunings(double(settings.pidKp), double(settings.pidKi), double(settings.pidKd), P_ON_M);            
        }

        if (iDryer.data.ntcTemp >= iDryer.temperature) //TODO:  првоерить что это целевая температура iDryer.temperature
        {
            if (millis() - oldTime > 1000)
            {
            oldTime = millis();
            Setpoint = iDryer.temperature + iDryer.data.ntcTemp - iDryer.data.bmeTemp;
            if (Setpoint > iDryer.temperature + 10) Setpoint = iDryer.temperature + 10;
            }
        }
        else
        {
        Setpoint = iDryer.temperature + 2;
        }

        if (iDryer.data.setHumidity > iDryer.data.bmeHumidity)
        {
            Input = double(iDryer.data.ntcTemp);
            myPID.Compute();

            // regulator.setpoint = iDryer.temperature;
            // regulator.input = iDryer.data.ntcTemp;

#ifdef v220V
            dimmer = map(Output, 0, 255, 500, 9300);
#endif

#ifdef v24V
            analogWrite(DIMMER_PIN, Output);
#endif
        }
        else
        {
            Input = double(iDryer.data.ntcTemp);
            Setpoint = 0;
            myPID.Compute();
#ifdef v220V
            dimmer = 0;
#endif

#ifdef v24V
            analogWrite(DIMMER_PIN, 0);
#endif
        }

        break;
    case AUTOPID:
        analogWrite(FAN, 200);
        PIDAutotuner tuner = PIDAutotuner();
        // tuner.setTargetInputValue(targetInputValue);
        tuner.setTargetInputValue(iDryer.data.setTemp);
        tuner.setLoopInterval(uint32_t(settings.pidDt) * 1000);
        tuner.setTuningCycles(20);
        tuner.setOutputRange(0, 255);
        tuner.setZNMode(PIDAutotuner::ZNModeNoOvershoot); // ZNModeNoOvershoot - Defaults,   ZNModeBasicPID
        tuner.startTuningLoop(micros());

        unsigned long microseconds;
        while (!tuner.isFinished())
        {
            unsigned long prevMicroseconds = microseconds;
            microseconds = micros();

            iDryer.getData();
            double output = tuner.tunePID(double(iDryer.data.ntcTemp), microseconds);

#ifdef v220V
            // dimmer = map(tuner.getOutput(), 0, 255, 500, 9300);
            dimmer = map(uint8_t(tuner.tunePID(double(iDryer.data.ntcTemp), microseconds)), 0, 255, 500, 9300);
#endif

#ifdef v24V
            analogWrite(DIMMER_PIN, uint8_t(tuner.tunePID(double(iDryer.data.ntcTemp), microseconds)));
#endif

            uint32_t new_loop = uint32_t(settings.pidDt) * 1000;

                // TODO чтонить про процесс пидования написать
                oled.firstPage();
                do
                {
                    char pidTxt[10];
                    oled.setFont(u8g2_font);
                    oled.drawUTF8(34, lineHight * 2, printMenuItem(&menuPGM[23].text));

                    sprintf(pidTxt, "C: %d  T: %d", tuner.getCycle(), (uint8_t)ntc.analog2temp());
                    oled.drawUTF8(10, lineHight * 1, pidTxt);

                    sprintf(pidTxt, "Кп %d", (uint8_t)tuner.getKp());
                    oled.drawUTF8(14, lineHight * 2, pidTxt);

                    sprintf(pidTxt, "Ки %d", (uint8_t)tuner.getKi());
                    oled.drawUTF8(14, lineHight * 3, pidTxt);

                    sprintf(pidTxt, "Кд %d", (uint8_t)tuner.getKd());
                    oled.drawUTF8(14, lineHight * 4, pidTxt);

                    // oled.drawUTF8(20, lineHight * 3, *pidTxt);
                    oled.drawButtonUTF8(0, 1 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
                    oled.drawButtonUTF8(0, 2 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
                    oled.drawButtonUTF8(0, 3 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
                    oled.drawButtonUTF8(0, 4 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
                } while (oled.nextPage());

            delay(settings.pidDt); //while (micros() - microseconds < new_loop)

            // Serial.println("ntc:" + String(ntc.analog2temp()));
            // Serial.print("\toutput:" + String(output));
            // Serial.println("\tkp:" + String(tuner.getKp()) + "\tki:" + String(tuner.getKi()) + "\tkd:" + String(tuner.getKd()));
            // Serial.println("ntc:" + String(ntc.analog2temp()) + "\toutput:" + String(output) + "\tkp:" + String(tuner.getKp()) + "\tki:" + String(tuner.getKi()) + "\tkd:" + String(tuner.getKd()));
        }
        // Serial.println(100);
        dimmer = 0;
        delay(100);
        analogWrite(DIMMER_PIN, 0);

        // Сохраняем
        settings.state = 123;
        settings.pidKp = tuner.getKp();
        settings.pidKi = tuner.getKi();
        settings.pidKd = tuner.getKd();
        settings.pidDt = settings.pidDt;
        EEPROM.put(0, settings);

        do
        {
            char pidTxt[10];
            oled.setFont(u8g2_font);
            oled.drawUTF8(34, lineHight * 2, printMenuItem(&menuPGM[23].text));
            sprintf(pidTxt, "СОХРАНЕНЕНО");
            oled.drawUTF8(28, lineHight * 3, pidTxt);
            // oled.drawUTF8(20, lineHight * 3, *pidTxt);
            oled.drawButtonUTF8(0, 1 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
            oled.drawButtonUTF8(0, 2 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
            oled.drawButtonUTF8(0, 3 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
            oled.drawButtonUTF8(0, 4 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
        } while (oled.nextPage());
        
        analogWrite(FAN, 250);
        delay(1000);

        do
        {
            oled.setFont(u8g2_font);
            char pidTxt[10];
            oled.drawUTF8(34, lineHight * 2, printMenuItem(&menuPGM[23].text));
            sprintf(pidTxt, "ОХЛАЖДЕНИЕ");
            oled.drawUTF8(28, lineHight * 3, pidTxt);
            sprintf(pidTxt, "60СЕКУНД");
            oled.drawUTF8(28, lineHight * 4, pidTxt);
            // oled.drawUTF8(20, lineHight * 3, *pidTxt);
            oled.drawButtonUTF8(0, 1 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
            oled.drawButtonUTF8(0, 2 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
            oled.drawButtonUTF8(0, 3 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
            oled.drawButtonUTF8(0, 4 * lineHight, U8G2_BTN_INV, 128, 0, 0, "");
        } while (oled.nextPage());

        delay(60000);

        state = MENU;
        break;
    default:
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
    uint8_t lineHight = 16;
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
    state = DRY;
    EEPROM.put(settingsSize, menuVal);
    iDryer.data.setTemp = menuVal[subMenuM.parentID + 1];
    iDryer.data.setTime = menuVal[subMenuM.parentID + 2];
    analogWrite(FAN, map(menuVal[24], 0, 100, 0, 255)); //!! Что-то придумать!!!

    iDryer.data.startTime = millis();

    // Serial.println("DRY START");
}

void storageStart()
{
    state = STORAGE;
    EEPROM.put(settingsSize, menuVal);
    iDryer.data.setTemp = menuVal[subMenuM.parentID + 1];
    iDryer.data.setHumidity = menuVal[subMenuM.parentID + 2];

    analogWrite(FAN, map(menuVal[24], 0, 100, 0, 255)); //!! Что-то придумать!!!
    // Serial.println("STORAGE START");
}

void autoPidM()
{
    state = AUTOPID;
}

void saveAll()
{
    EEPROM.put(0, settings);
    EEPROM.put(settingsSize, menuVal);
    // Serial.println("SAVE ALL");
}