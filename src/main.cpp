//!* OLED https://startingelectronics.org/tutorials/arduino/modules/OLED-128x32-I2C-display/
//!* BME280 https://randomnerdtutorials.com/bme280-sensor-arduino-pressure-temperature-humidity/
//!* https://github.com/AlexGyver/tutorials/tree/master/triac/smooth_1ch
//!! https://github.com/AlexGyver/AC_Dimmer  схемы и всякое такое
//!* https://alexgyver.ru/lessons/pid/
//!* https://github.com/GyverLibs/GyverPID/blob/main/examples/autotune2/
//!* https://alexgyver.ru/lessons/eeprom/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <U8g2lib.h>
#include <GyverTimers.h> // библиотека таймера
#include <GyverNTC.h>
#include "GyverPID.h"
#include "PIDtuner2.h"
#include "thermistorMinim.h"
#include <EEPROM.h>
#include <GyverEncoder.h>
#include <EncButton.h>
// Энкодер https://alexgyver.ru/encoder/
#define SW 5  // Pin encoder Button
#define DT 6  // Pin  Detect
#define CLK 7 // Pin  Clockwise
// Энкодер

#define MAX_TEMP 100
// Димер
#define ZERO_PIN 2   // пин детектора нуля
#define INT_NUM 0    // соответствующий ему номер прерывания
#define DIMMER_PIN 4 // управляющий пин симистора
// Димер
#define buzzerPin 10
#define FAN 11

#define NTC_PIN 0
#define SEALEVELPRESSURE_HPA (1013.25) // оценивает высоту в метрах на основе давления на уровне моря
#define HESTERESIS 2                   // оценивает высоту в метрах на основе давления на уровне моря

#define OFF 0
#define ON 1
#define DRY 2
#define STORAGE 3
#define AUTOPID 4
#define NTC_ERROR 5

U8G2_SSD1306_128X64_NONAME_1_HW_I2C oled(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
Adafruit_BME280 bme;
Encoder enc(CLK, DT, SW, TYPE1);
GyverNTC ntc(NTC_PIN, 10000, 3950);
PIDtuner2 tuner;

//!! взять из автотюна
GyverPID regulator(0.1, 0.05, 0.01, 10);

int dimmer; // переменная диммера
int dryTemp = 0;
uint64_t timeToDry = 0;
uint64_t startDry = 0;
uint16_t timer = 0;
uint16_t storageTemp = 35;
uint8_t pid_itr = 0;
bool autoPid = OFF;
uint8_t state = 0;

//!* menu
int reset = LOW;
int select = 0;
int exitMenu = LOW;
int first = LOW;
int item = 0;
int on = 1;
int epr = 0;
//!* menu

char str1[12];
char str2[12];
char str3[12];
char str4[12];

struct Settings
{
    bool sate = 0;
    float pidKp = 0;
    float pidKi = 0;
    float pidKd = 0;
    int pidDt = 0;
    int storageTemp = 35;
    int dryTemp = 60;
    int timeToDry = 4 * 60 * 60 * 1000;
};
Settings settings;

struct Data
{
    int ntcTemp = 0;
    float bmeTemp = 0;
    float bmeHumidity = 0;
    int timer = 0;
};

class iDryer
{
public:
    int ntcTemp = 0;
    float bmeTemp = 0;
    float bmeHumidity = 0;
    int timer = 0;
    bool newDataFlag = false;
    int temperature = 0;
    Data oldData;
    Data data;

    bool getData(int setTemperature)
    {
        data.ntcTemp = ntc.getTempAverage();
        data.bmeTemp = bme.readTemperature();
        data.bmeHumidity = bme.readHumidity();

        if (data.bmeTemp < setTemperature - HESTERESIS)
            temperature = setTemperature + 10;
        if (data.bmeTemp > setTemperature + HESTERESIS)
            temperature = setTemperature + 5;
        if (data.bmeTemp > setTemperature + HESTERESIS)
            temperature = 0;
        if (data.bmeTemp > setTemperature - HESTERESIS && data.bmeTemp < setTemperature + HESTERESIS)
            temperature = setTemperature;

        if (data.ntcTemp != oldData.ntcTemp ||
            data.bmeTemp != oldData.bmeTemp ||
            data.bmeHumidity != oldData.bmeHumidity)
        {
            oldData = data;
            newDataFlag = true;
            return newDataFlag;
        }
        newDataFlag = false;
        return newDataFlag;
    }
} iDryer;

void isrCLK()
{
    enc.tick(); // отработка в прерывании
}
void isrDT()
{
    enc.tick(); // отработка в прерывании
}

//* ОБРАБОТКА ЭНКОДЕРА - ДЛЯ МЕНЮ
void rotaryMenu(int *s, int *i)
{
    if (enc.isRight())
    {
        *s = *s + 1;
        if (*s >= *i)
            *s = 0;
    }
    if (enc.isLeft())
    {
        *s = *s - 1;
        if (*s < 0)
            *s = *i - 1;
    }
}



void storage(int *s, int *e, int *f)
{
    bool exit1 = OFF;
    do
    {
        switch (*s)
        {
        case 0:
            /* code */
            break;

        default:
            break;
        }
    } while (exit1 == OFF);
}

// прерывание детектора нуля
void isr()
{
    static int lastDim;
    digitalWrite(DIMMER_PIN, 0); // выключаем симистор
    // если значение изменилось, устанавливаем новый период
    // если нет, то просто перезапускаем со старым
    if (lastDim != dimmer)
        Timer1.setPeriod(lastDim = dimmer);
    else
        Timer1.restart();
}

// прерывание таймера
ISR(TIMER1_A)
{
    digitalWrite(DIMMER_PIN, 1); // включаем симистор
    Timer1.stop();               // останавливаем таймер
}

void dispalyPrint4(char *STR1, char *STR2, char *STR3, char *STR4)
{
    // Serial.print("     пришло:");
    // Serial.println(STR3);
    uint8_t width1 = oled.getUTF8Width(STR1);
    uint8_t width2 = oled.getUTF8Width(STR2);
    uint8_t width3 = oled.getUTF8Width(STR3);
    uint8_t width4 = oled.getUTF8Width(STR4);
    oled.firstPage();
    do
    {
        oled.setFont(u8g2_font_VCR_OSD_tu); // draw the current pixel width
        if (width1 > 0)
        {
            oled.drawUTF8((128 - width1) / 2, 16, STR1);
        }
        if (width2 > 0)
        {
            oled.drawUTF8((128 - width2) / 2, 32, STR2);
        }
        if (width3 > 0)
        {
            oled.drawUTF8((128 - width3) / 2, 48, STR3);
        }
        if (width4 > 0)
        {
            oled.drawUTF8((128 - width4) / 2, 64, STR4);
        }
    } while (oled.nextPage());
}

void menu(int *s, int *e, int *f)
{
    switch (*s)
    {
    case 0:
        dispalyPrint4("", "DRY", "MODE", "");
        if (enc.isClick())
        {
            item = 4;
            rotaryMenu(&select, &item);
            // dry(&select, &exitMenu, &first);
        }
        break;

    case 1:
        dispalyPrint4("", "STORAGE", "MODE", "");
        if (enc.isClick())
        {
            storage(&select, &exitMenu, &first);
        }
        break;

    case 2:
        dispalyPrint4("", "AUTO PID", "click for", " start");
        if (enc.isClick())
        {
            state = AUTOPID;
        }
        break;
    case 3:
        dispalyPrint4("", "EXIT", "MENU", "");
        if (enc.isClick())
        {
            *e = HIGH;
        }
        break;
    }
    *f = OFF;
}

void dry(int *s, int *e, int *f)
{
    bool exit1 = OFF;
    do
    {
        switch (*s)
        {
        case 0:
            /* code */
            break;

        default:
            break;
        }
    } while (exit1 == OFF);
}

void setup()
{
    EEPROM.get(0, settings);
    if (settings.sate)
    {
        regulator.Kd = settings.pidKd;
        regulator.Kp = settings.pidKp;
        regulator.Ki = settings.pidKi;
        // https://alexgyver.ru/gyverpid/
        // установить пределы Время итерации: время итерации можно изменить в процессе
        // работы (не знаю, зачем, но возможность есть). Время устанавливается в миллисекундах
        // и влияет на функцию getResultTimer(), которая с этим периодом делает новый расчёт
        // управляющего сигнала. Также это время входит в расчёт управляющего
        // сигнала (в И и Д составляющей). Устанавливается командой setDt(dt);  // установка времени итерации в мс
        regulator.setDt(settings.pidDt); // время

        dryTemp = settings.storageTemp;
        timeToDry = settings.timeToDry;
    }

    // Диммер
    pinMode(ZERO_PIN, INPUT_PULLUP);
    pinMode(DIMMER_PIN, OUTPUT);

    pinMode(DT, INPUT_PULLUP);
    pinMode(CLK, INPUT_PULLUP);
    pinMode(SW, INPUT_PULLUP);
    pinMode(buzzerPin, OUTPUT);

    attachInterrupt(INT_NUM, isr, RISING); // для самодельной схемы ставь FALLING
    Timer1.enableISR();                    // Timer2.enableISR();
    // Диммер

    enc.setType(TYPE2);
    attachInterrupt(0, isrCLK, CHANGE); // прерывание на 2 пине! CLK у энка
    attachInterrupt(1, isrDT, CHANGE);  // прерывание на 3 пине! DT у энка
    Serial.begin(9600);

    if (!bme.begin(0x76))
    {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1)
            ;
    }

    regulator.setDirection(NORMAL); // направление регулирования (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
    regulator.setLimits(0, 255);    // пределы (ставим для 8 битного ШИМ). ПО УМОЛЧАНИЮ СТОЯТ 0 И 255
    regulator.setpoint = 50;        // сообщаем регулятору температуру, которую он должен поддерживать
    // // в процессе работы можно менять коэффициенты
    // regulator.Kp = 5.2;
    // regulator.Ki += 0.5;
    // regulator.Kd = 0;

    oled.begin();
    oled.setFlipMode(1);
    oled.setContrast(0);
    delay(200);
    dispalyPrint4("-", "-", "-", "-"); //!!!! НЕ УДАЛЯТЬ ИНАЧЕ СТАРТУЕТ КРИВО, ВИДИМО НУЖНО СНАЧАЛА ПНУТЬ ЭКРАН
    dispalyPrint4("SUPER", "PUPER", "SUSHILKA", "BY ENGENNER");
    tone(buzzerPin, 50);
    delay(50);
    // delay(3000);
}

void loop()
{
//!*

    if (iDryer.getData(dryTemp))
    {
        sprintf(str1, "air t:  %.2f C", iDryer.data.bmeTemp);
        sprintf(str2, "air H:  %03d %", iDryer.data.bmeHumidity);
        sprintf(str3, "bed t: %.2f C", iDryer.data.ntcTemp);
        sprintf(str4, "timer: %03d C", iDryer.data.timer);
        Serial.println(str1);
        Serial.println(str2);
        Serial.println(str3);
        Serial.println(str4);
    }

    enc.tick();

    int tmpTemp = analogRead(NTC_PIN);
    if (tmpTemp == 0 || tmpTemp >= 1023)
    {
        state = NTC_ERROR;
    }

    if (enc.isHold())
    {
        delay(700);
        dispalyPrint4("", "", "", "RELEASE IT");
        delay(500);
        do
        {
            item = 4; //////////////////// item main menu
            rotaryMenu(&select, &item);
            menu(&select, &exitMenu, &first);
        } while (exitMenu == LOW);
        delay(300);
    }

    // МЕНЮ
    //-Сушка
    //--Температура
    //--Время
    //--Старт
    //-Подогрев
    //--Температура
    //--Старт
    //-Автопид
    //--Старт
    //-?

    switch (state)
    {
    case NTC_ERROR:
        dimmer = 0;
        digitalWrite(DIMMER_PIN, 0);
        detachInterrupt(INT_NUM);
        analogWrite(FAN, 255);
        while (1)
        {
            dispalyPrint4("", "CHECK NTC", "AND RESTART", "");
            tone(buzzerPin, 500, 500);
            dispalyPrint4("", "ERROR", "ERROR", "");
            tone(buzzerPin, 1000, 500);
        }
        // state = OFF;
        break;
    case OFF:
        /* code */
        break;
    case ON:

        break;
    case DRY:
        // sprintf(bmePresureChar, "P: %04d mm", bme.readPressure() / 100.0F);
        // sprintf(bmeAltitude, "A: %04d m", bme.readAltitude(SEALEVELPRESSURE_HPA));

        if (iDryer.getData(dryTemp))
        {
            sprintf(str1, "air t:  %.2f C", iDryer.data.bmeTemp);
            sprintf(str2, "air H:  %03d %", iDryer.data.bmeHumidity);
            sprintf(str3, "bed t: %.2f C", iDryer.data.ntcTemp);
            sprintf(str4, "timer: %03d C", iDryer.data.timer);
            dispalyPrint4(str1, str2, str3, str4);
        }
        regulator.setpoint = iDryer.temperature;
        regulator.input = iDryer.data.ntcTemp;
        dimmer = map(regulator.getResultTimer(), 0, 255, 500, 9300);
        if (millis() - startDry > timeToDry)
        {
            dryTemp = storageTemp;
        }
        else
        {
            state = OFF;
        }
        break;
    case STORAGE:
        if (iDryer.getData(storageTemp))
        {
            sprintf(str1, "STORAGE");
            sprintf(str2, "air t:  %.2f C", iDryer.data.bmeTemp);
            sprintf(str3, "air H:  %03d %", iDryer.data.bmeHumidity);
            sprintf(str4, "bed t: %.2f C", iDryer.data.ntcTemp);
            dispalyPrint4(str1, str2, str3, str4);
        }
        regulator.setpoint = iDryer.temperature;
        regulator.input = iDryer.data.ntcTemp;
        dimmer = map(regulator.getResultTimer(), 0, 255, 500, 9300);
        break;
    case AUTOPID:
        while (pid_itr < 7) // AUTOPID
        {
            // направление, начальный сигнал, конечный, период плато, точность, время стабилизации, период итерации
            tuner.setParameters(NORMAL, 0, 80, 6000, 0.05, 500);

            tuner.setInput(ntc.getTempAverage());
            tuner.compute();
            dimmer = map(tuner.getOutput(), 0, 255, 500, 9300);
            if (tuner.getState() != 7 && pid_itr != tuner.getState())
            {
                pid_itr = tuner.getState();
                sprintf(str1, "PID ATOTUNE");
                sprintf(str2, "Kp:  %.2f", tuner.getPID_p());
                sprintf(str3, "Ki:  %.2f", tuner.getPID_i());
                sprintf(str4, "Kd:  %.2f", tuner.getPID_d());
                dispalyPrint4(str1, str2, str3, str4);
            }
            if (tuner.getState() == 7)
            {
                settings.pidKp = tuner.getPID_p(); // p для ПИД регулятора
                settings.pidKi = tuner.getPID_i(); // i для ПИД регулятора
                settings.pidKd = tuner.getPID_d(); //  d для ПИД регулятора
                dimmer = 0;
                //!* запихнуть все в епром
                //!! EEPROM.put(0, settings);
                //!! delay(50);
                dispalyPrint4("PID", "IS", "COMPUTE", "AND SAVE");
                tone(buzzerPin, 500, 100);
                delay(100);
                tone(buzzerPin, 500, 1000);
                sprintf(str1, "CURRENT PID");
                sprintf(str2, "Kp:  %.2f", tuner.getPID_p());
                sprintf(str3, "Ki:  %.2f", tuner.getPID_i());
                sprintf(str4, "Kd:  %.2f", tuner.getPID_d());
                dispalyPrint4(str1, str2, str3, str4);
                state = OFF;
            }
        }
        break;

    default:
        break;
    }
}