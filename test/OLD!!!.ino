#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
// #include "thermistorMinim.h"
#include <EEPROM.h>
#include <GyverTimers.h> // библиотека таймера
#include <GyverEncoder.h>
#include <GyverBME280.h>
#include "GyverPID.h"
#include "PIDtuner2.h"
// #include <GyverNTC.h>
#include <thermistor.h>

/**
 * --NORMAL IS 4.7kohm PULLUP!-- 1kohm pullup can be used on hotend sensor, using correct resistor and table
 *
 * Temperature sensors available:
 *
 *    -3 : thermocouple with MAX31855 (only for sensor 0)
 *    -2 : thermocouple with MAX6675 (only for sensor 0)
 *    -1 : thermocouple with AD595
 *     0 : not used
 *     1 : 100k thermistor - best choice for EPCOS 100k (4.7k pullup)
 *     2 : 200k thermistor - ATC Semitec 204GT-2 (4.7k pullup)
 *     3 : Mendel-parts thermistor (4.7k pullup)
 *     4 : 10k thermistor !! do not use it for a hotend. It gives bad resolution at high temp. !!
 *     5 : 100K thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (4.7k pullup)
 *     6 : 100k EPCOS - Not as accurate as table 1 (created using a fluke thermocouple) (4.7k pullup)
 *     7 : 100k Honeywell thermistor 135-104LAG-J01 (4.7k pullup)
 *    71 : 100k Honeywell thermistor 135-104LAF-J01 (4.7k pullup)
 *     8 : 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup)
 *     9 : 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)
 *    10 : 100k RS thermistor 198-961 (4.7k pullup)
 *    11 : 100k beta 3950 1% thermistor (4.7k pullup)
 *    12 : 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup) (calibrated for Makibox hot bed)
 *    13 : 100k Hisens 3950  1% up to 300°C for hotend "Simple ONE " & "Hotend "All In ONE"
 *    20 : the PT100 circuit found in the Ultimainboard V2.x
 *    60 : 100k Maker's Tool Works Kapton Bed Thermistor beta=3950
 *    66 : 4.7M High Temperature thermistor from Dyze Design
 *    70 : the 100K thermistor found in the bq Hephestos 2
 *    75 : 100k Generic Silicon Heat Pad with NTC 100K MGB18-104F39050L32 thermistor
 *
 *       1k ohm pullup tables - This is atypical, and requires changing out the 4.7k pullup for 1k.
 *                              (but gives greater accuracy and more stable PID)
 *    51 : 100k thermistor - EPCOS (1k pullup)
 *    52 : 200k thermistor - ATC Semitec 204GT-2 (1k pullup)
 *    55 : 100k thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (1k pullup)
 *
 *  1047 : Pt1000 with 4k7 pullup
 *  1010 : Pt1000 with 1k pullup (non standard)
 *   147 : Pt100 with 4k7 pullup
 *   148 : E3D Pt100 with 4k7 pullup
 *   110 : Pt100 with 1k pullup (non standard)
 *
 *         Use these for Testing or Development purposes. NEVER for production machine.
 *   998 : Dummy Table that ALWAYS reads 25°C or the temperature defined below.
 *   999 : Dummy Table that ALWAYS reads 100°C or the temperature defined below.
// * :{ '0': "Not used", '1':"100k / 4.7k - EPCOS", '2':"200k / 4.7k - ATC Semitec 204GT-2", '3':"Mendel-parts / 4.7k", '4':"10k !! do not use for a hotend. Bad resolution at high temp. !!", '5':"100K / 4.7k - ATC Semitec 104GT-2 (Used in ParCan & J-Head)", '6':"100k / 4.7k EPCOS - Not as accurate as Table 1", '7':"100k / 4.7k Honeywell 135-104LAG-J01", '8':"100k / 4.7k 0603 SMD Vishay NTCS0603E3104FXT", '9':"100k / 4.7k GE Sensing AL03006-58.2K-97-G1", '10':"100k / 4.7k RS 198-961", '11':"100k / 4.7k beta 3950 1%", '12':"100k / 4.7k 0603 SMD Vishay NTCS0603E3104FXT (calibrated for Makibox hot bed)", '13':"100k Hisens 3950  1% up to 300°C for hotend 'Simple ONE ' & hotend 'All In ONE'", '20':"PT100 (Ultimainboard V2.x)", '51':"100k / 1k - EPCOS", '52':"200k / 1k - ATC Semitec 204GT-2", '55':"100k / 1k - ATC Semitec 104GT-2 (Used in ParCan & J-Head)", '60':"100k Maker's Tool Works Kapton Bed Thermistor beta=3950", '66':"Dyze Design 4.7M High Temperature thermistor", '70':"the 100K thermistor found in the bq Hephestos 2", '71':"100k / 4.7k Honeywell 135-104LAF-J01", '147':"Pt100 / 4.7k", '148':"E3D Pt100 / 4.7k", '1047':"Pt1000 / 4.7k", '110':"Pt100 / 1k (non-standard)", '1010':"Pt1000 / 1k (non standard)", '-3':"Thermocouple + MAX31855 (only for sensor 0)", '-2':"Thermocouple + MAX6675 (only for sensor 0)", '-1':"Thermocouple + AD595",'998':"Dummy 1", '999':"Dummy 2" }
*/

//!! УСТНОВИТЬ ТИП ТЕРМИСТОРА
#define TEMP_SENSOR_0 1

// #define DT 2     // Pin  Detect 7
// #define CLK 3    // Pin  Clockwise  6
// #define encBut 5 // Pin encoder Button

#define NTC_PIN 0
// #define SEALEVELPRESSURE_HPA (1013.25) // оценивает высоту в метрах на основе давления на уровне моря

#define HESTERESIS 2
// Димер
#define ZERO_PIN 2   // пин детектора нуля
#define DIMMER_PIN 4 // управляющий пин симистора
#define INT_NUM 0    // соответствующий ему номер прерывания
// Димер

#define buzzerPin 10
#define FAN 11

// #define screenSizeV 64 // !! Заменить screenSizeY
// #define screenSizeX 64 // !! Заменить 128 на переменную
// #define fontHight 15

#define SCREEN_LINES 4
#define MENU_HEADER 1

#define OFF 0
#define ON 1
#define MENU 2
#define DRY 3
#define STORAGE 4
#define AUTOPID 5
#define NTC_ERROR 6

#define MAX_ERROR 10

#define TMP_MIN 10
#define TMP_MAX 120

#define ADC_MIN 200
#define ADC_MAX 1000

const uint8_t u8g2_font[718] U8G2_FONT_SECTION("u8g2_font") = 
  "+\0\3\2\5\5\3\2\6\20\20\0\376\16\376\16\376\0\0\0\0\0\263\0#\20\22\30I\277\223t"
  "N:\251\22%\331\220\224\222\64Q\242$\233\42i\310I:'\235\223NJ\372\1\60\21FUTZ"
  "\224\204\332\224(\321&&Q&\1\61\13EYT\231\224\204}\32\4\62\16FU\64C\22\212i\246"
  "\205\325t\30\63\20FU\64C\22\212i\64\247\242\230\14\11\0\64\20FU\224\241\226D\225,\311\222"
  "aL+\0\65\16FU\24\207\264:\310i*&C\2\66\16FUTS\230\246\203\22:&C\2"
  "\67\13FU\24\327bZL\233\0\70\17FU\64C\22\32\223!\11\35\223!\1\71\16FU\64C"
  "\22\32\223Am\214&\0\0\0\0\4\377\377\4\20\16FUTZ\324\22\212\303 :\6\4\21\17F"
  "U\24\203\222V\7%t\34\26\0\4\22\17FU\24\203\22\32\207%t\34\26\0\4\23\12FU\24"
  "\207\264\277\2\4\24\23\210\21\224c\24Fa\224\365)\214\302d\70\313\1\4\25\15FU\24\207\264:"
  "(i\353\60\4\26\23GU\24Q$U\232\222\332\270%\245\244)\222\12\4\27\21FU\64C\22\246"
  "\225!\215S\61\31\22\0\4\30\21FU\24\241vJ\244D\211\224h\333\304\0\4\31\24\246U\64Q"
  "\246#\241vJ\244D\211\224h\333\304\0\4\32\21FU\24\331\324%K\62\61\311\242Z\22\6\4\33"
  "\15FUTC\324\377%K\62\61\4\34\15FU\24\241\70\15\321\342\321\61\4\35\14FU\24\241\343"
  "\60\210\36\3\4\36\15FU\64C\22\372c\62$\0\4\37\12FU\24\207\320\77\6\4 \15FU"
  "\24\203\22\32\207%\355\12\4!\16FU\64C\22Z;\212\311\220\0\4\42\13GU\24\207,\356o"
  "\0\4#\21GU\24\251\232dQV\11\223\64NS\21\4$\24gUt\341\240D\221\24I\221\24"
  "I\225A\214k\0\4%\20FU\24\241\230Dm\242\26\265\204b\0\4&\27\207\25\24a\22&a"
  "\22&a\22&a\22&a\62\14q\1\4'\13FU\24\241o\312\222\66\4(\26GU\24Q$"
  "ER$ER$ER$ER\64\14\2\4)\32\210\21\24Q%\252D\225\250\22U\242JT\211"
  "*Qe\30t \7\2\4*\21GU\24s\363\220\225\262(\213\262hH\0\4+\22FU\24\241"
  "\343\20I\211\224H\211\224\14Q\0\4,\15FU\24i\353\240\204\216\303\2\4-\17FU\64S\26"
  "\247\311\240V\265h\2\4.\25FU\24\221RR\42%\32\42%R\42%R\42\13\0\4/\20F"
  "U\64\303hL\6\251\222%\231(\6\0";


uint8_t lineHight = 16;
uint16_t dimmer;
uint64_t startDry = 0;
uint16_t timer = 0;
uint8_t pid_itr = 0;
bool autoPid = OFF;
uint8_t state = MENU;
uint8_t ERROR_COUNTER = 0;

uint8_t testPWM = 0;
uint16_t testTIMER_COUNT = 0;

const byte CLK = A1;
const byte DT = A2;
const byte encBut = A3;
volatile static byte oldPorta;
volatile static byte PCMask;
volatile byte flagISR = 0, intsFound = 0;
byte isr_1 = 0;
byte isr_2 = 0;
byte isr_3 = 0;

//!* menu
uint8_t menuSize = 0;
//!* menu

enum levelV
{
    NOTHING,
    DOWN,
    UP,
};

void testFuc_1()
{
    Serial.println("УРРРРА\nЗА\nРА\nБО\nТА\nЛА\n");
}
void dryStart()
{
    state = DRY;
    Serial.println("DRY START");
    delay(400);
    // TODO: ВЫВОД НА ЭКРАН
}
void storageStart()
{
    state = STORAGE;
    // TODO: ВЫВОД НА ЭКРАН
    Serial.println("STORAGE START");
    delay(400);
}

void changeTermistor() {}
void autoPidM()
{
    state = AUTOPID;
}

typedef void (*ptrFunc)();

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

const char i0[] PROGMEM = "МЕНЮ";
const char i1[] PROGMEM = "СУШКА";
const char i2[] PROGMEM = "ТЕМП.";
const char i3[] PROGMEM = "ВРЕМЯ";
const char i4[] PROGMEM = "СТАРТ";
const char i5[] PROGMEM = "ХРАНЕНИЕ";
const char i6[] PROGMEM = "ТЕМП.";
const char i7[] PROGMEM = "СТАРТ";
const char i8[] PROGMEM = "ПРЕСЕТЫ";
const char i9[] PROGMEM = "PLA";
const char i10[] PROGMEM = "СТАРТ";
const char i11[] PROGMEM = "ТЕМП.";
const char i12[] PROGMEM = "ВРЕМЯ";
const char i13[] PROGMEM = "PETG";
const char i14[] PROGMEM = "СТАРТ";
const char i15[] PROGMEM = "ТЕМП.";
const char i16[] PROGMEM = "ВРЕМЯ";
const char i17[] PROGMEM = "ABS";
const char i18[] PROGMEM = "СТАРТ";
const char i19[] PROGMEM = "ТЕМП.";
const char i20[] PROGMEM = "ВРЕМЯ";
const char i21[] PROGMEM = "НАСТРОЙКИ";
const char i22[] PROGMEM = "ТЕРМИСТОР";
const char i23[] PROGMEM = "pt100";
const char i24[] PROGMEM = "pt1000";
const char i25[] PROGMEM = "АВТОПИД";
const char i26[] PROGMEM = "1НАСТ";

const char s0[] PROGMEM = "СУШКА";
const char s1[] PROGMEM = "ХРАНЕНИЕ";
const char l0[] PROGMEM = "ВОЗДУХ";
const char l1[] PROGMEM = "НАГРЕВАТЕЛЬ";
const char l2[] PROGMEM = "ВЛАЖНОСТЬ";
const char l3[] PROGMEM = "ВЛАЖНОСТЬ";

const char *const txt[] PROGMEM = {
    s0,
    s1,
    l0,
    l1,
    l2,
    l3};

struct menuS
{
    uint8_t id;
    uint8_t parentID;
    uint8_t level;
    const char *text;
    uint16_t min;
    uint16_t max;
    ptrFunc action;
};

const menuS menuPGM[] PROGMEM = {
    {0, NULL, 0, i0, 0, 0, NULL},
    {1, 0, 1, i1, 0, 0, NULL},
    {2, 1, 2, i2, 0, 100, NULL},
    {3, 1, 2, i3, 0, 600, NULL},
    {4, 1, 2, i4, 0, 0, &dryStart},
    {5, 0, 1, i5, 0, 0, NULL},
    {6, 5, 2, i6, 0, 100, NULL},
    {7, 5, 2, i7, 0, 0, &storageStart},
    {8, 0, 1, i8, 0, 0, NULL},
    {9, 8, 2, i9, 0, 0, NULL},
    {10, 9, 3, i10, 0, 0, &dryStart},
    {11, 9, 3, i11, 45, 100, NULL},
    {12, 9, 3, i12, 120, 600, NULL},
    {13, 8, 2, i13, 0, 0, NULL},
    {14, 13, 3, i14, 0, 0, &dryStart},
    {15, 13, 3, i15, 45, 100, NULL},
    {16, 13, 3, i16, 120, 600, NULL},
    {17, 8, 2, i17, 0, 0, NULL},
    {18, 17, 3, i18, 0, 0, &dryStart},
    {19, 17, 3, i19, 45, 100, NULL},
    {20, 17, 3, i20, 120, 600, NULL},
    {21, 0, 1, i21, 0, 0, NULL},
    {22, 21, 2, i22, 0, 0, NULL},
    {23, 22, 3, i23, 0, 0, &changeTermistor},
    {24, 22, 3, i24, 0, 0, &changeTermistor},
    {25, 21, 2, i25, 0, 0, &autoPidM},
    {26, 0, 1, i26, 0, 0, NULL},
};



uint16_t menuVal[] =
    {
        0,
        0,
        60,
        240,
        0,
        0,
        35,
        0,
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
        100,
        1000,
        0,
        0,
};

struct Settings
{
    bool state = 0;
    float pidKp = 0;
    float pidKi = 0;
    float pidKd = 0;
    uint16_t pidDt = 0;
    // uint8_t storageTemp = 35;
    // uint8_t dryTemp = 60;
    // uint16_t timeToDry = 4 * 60 * 60 * 1000;
};

// GyverNTC ntc(NTC_PIN, 10000, 3950);
thermistor ntc(NTC_PIN,0);
GyverBME280 bme;

struct Data
{
    uint8_t ntcTemp = 0;
    float bmeTemp = 0;
    float bmeHumidity = 0;
    int timer = 0;
};

class iDryer
{
public:
    // int timer = 0;
    // bool newDataFlag = false;
    uint8_t temperature = 0;
    // Data oldData;
    Data data;

    bool getData(int setTemperature)
    {
        data.ntcTemp = (uint8_t)ntc.analog2temp();
        // data.ntcTemp = ntc.getTemp();
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

        if (data.ntcTemp < TMP_MIN||
            data.ntcTemp > TMP_MAX ||
            data.bmeTemp < TMP_MIN ||
            data.bmeTemp > TMP_MAX)
        {
            return false;
        }
        return true;
    }
} iDryer;

control controls;
subMenu subMenuM;
Settings settings;

// U8G2_SH1106_128X64_NONAME_1_HW_I2C oled(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
U8G2_SSD1306_128X64_NONAME_1_HW_I2C oled(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
Encoder enc(CLK, DT, encBut, TYPE1);
GyverPID regulator(0.1, 0.05, 0.01, 10);
PIDtuner2 tuner;

// void isrCLK()
// {
//     enc.tick(); // отработка в прерывании
// }

// void isrDT()
// {
//     enc.tick(); // отработка в прерывании
// }

void isr()
{
    testTIMER_COUNT++;
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

char *printMenuItem(const char *const *text) // печать строки из prm
{
    static char buffer[20]; // Буфер на полную строку
    //!! uint16_t ptr = pgm_read_word(&(names[num]));
    uint16_t ptr = pgm_read_word(text); // Получаем указатель на первый символ строки
    uint8_t i = 0;                      // Переменная - счетчик

    do
    {                                             // Начало цикла
        buffer[i] = (char)(pgm_read_byte(ptr++)); // Прочитать в буфер один символ из PGM и подвинуть указатель на 1
                                                  // } while (buffer[i++] != NULL);              // Если это не конец строки - вернуться в начало цикла
    } while (i++ != 20);                          // Если это не конец строки - вернуться в начало цикла
                                                  // Serial.println(buffer);
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
                    oled.drawButtonUTF8(0, lineHight * (i + 2) - 1, U8G2_BTN_INV, 128, 1, 1, "");
                }
                else
                {
                    oled.drawButtonUTF8(96, lineHight * (i + 2) - 1, U8G2_BTN_INV, 128, 1, 1, "");
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
        sprintf(val, "%3d", iDryer.data.timer);
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
void controlsHandler(const menuS constMenu[], uint16_t editableMenu[], struct control *encoder, struct subMenu *subMenu);
void screen(struct subMenu *subMenu);
void submenuHandler(const menuS constMenu[], uint8_t menuSize, struct subMenu *subMenu);

ISR( PCINT1_vect ) {
  byte change, v1, v2, v3;
  change = oldPorta ^ PINC;

  v1 = oldPorta & (1<<PCINT9);
  v2 = oldPorta & (1<<PCINT10);
  v3 = oldPorta & (1<<PCINT11);

  if (v1 == 0 && change & (1<<PCINT9) ) isr_1 = 1;
  if (v2 == 0 && change & (1<<PCINT10)) isr_2 = 1;
  if (v3 == 0 && change & (1<<PCINT11)) isr_3 = 1;
  if (change && (v1 == 0 || v2 == 0 || v3 == 0) ) {
    flagISR = 1;
    enc.tick();
    isr_1 = isr_2 = isr_3 = 0;
  }
  oldPorta = PINC;
}

void setup()
{
    Serial.begin(57600);

    EEPROM.get(0, settings);
    if (settings.state)
    {
        // regulator.Kd = settings.pidKd;
        // regulator.Kp = settings.pidKp;
        // regulator.Ki = settings.pidKi;
        // // https://alexgyver.ru/gyverpid/
        // // установить пределы Время итерации: время итерации можно изменить в процессе
        // // работы (не знаю, зачем, но возможность есть). Время устанавливается в миллисекундах
        // // и влияет на функцию getResultTimer(), которая с этим периодом делает новый расчёт
        // // управляющего сигнала. Также это время входит в расчёт управляющего
        // // сигнала (в И и Д составляющей). Устанавливается командой setDt(dt);  // установка времени итерации в мс
        // regulator.setDt(settings.pidDt); // время

        // dryTemp = settings.storageTemp;
        // timeToDry = settings.timeToDry;
    }

    EEPROM.get(0, menuVal);
    if (menuVal[2] || menuVal[3])
    {
        //TODO ЗАИПИСЬ
    }
    else
    {
        menuVal[0] = 0;
        menuVal[1] = 0;
        menuVal[2] = 60;
        menuVal[3] = 240;
        menuVal[4] = 0;
        menuVal[5] = 0;
        menuVal[6] = 35;
        menuVal[7] = 0;
        menuVal[8] = 0;
        menuVal[9] = 0;
        menuVal[10] = 0;
        menuVal[11] = 50;
        menuVal[12] = 240;
        menuVal[13] = 0;
        menuVal[14] = 0;
        menuVal[15] = 50;
        menuVal[16] = 240;
        menuVal[17] = 0;
        menuVal[18] = 0;
        menuVal[19] = 50;
        menuVal[20] = 240;
        menuVal[21] = 0;
        menuVal[22] = 0;
        menuVal[23] = 100;
        menuVal[24] = 1000;
        menuVal[25] = 0;
        menuVal[26] = 0;
    }

    // Диммер
    pinMode(ZERO_PIN, INPUT_PULLUP);
    pinMode(DIMMER_PIN, OUTPUT);

    pinMode(DT, INPUT_PULLUP);
    pinMode(CLK, INPUT_PULLUP);
    pinMode(encBut, INPUT_PULLUP);

    oldPorta = PINC;
    PCMSK1 |= (1<<PCINT9);
    PCMSK1 |= (1<<PCINT10);
    PCMSK1 |= (1<<PCINT11);
    Serial.println(PCMSK1,HEX);
    PCMask = PCMSK1;
    PCICR |= (1<<PCIE1);
    
    pinMode(buzzerPin, OUTPUT);

    attachInterrupt(INT_NUM, isr, RISING); // для самодельной схемы ставь FALLING
    Timer1.enableISR();                    // Timer2.enableISR();

    oled.begin();
    oled.setFlipMode(1);
    // oled.setContrast(1);
    oled.enableUTF8Print();
    oled.setFont(u8g2_font); //u8g2_font   u8g2_font

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
    // attachInterrupt(0, isrCLK, CHANGE);
    // attachInterrupt(1, isrDT, CHANGE);

    if (!bme.begin(0x76))
    {
        // Serial.println("Could not find a valid BME280 sensor, check wiring!");
        // while (1)
        ;
    }

      regulator.setDirection(NORMAL); // направление регулирования (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
      regulator.setLimits(0, 255);    // пределы (ставим для 8 битного ШИМ). ПО УМОЛЧАНИЮ СТОЯТ 0 И 255
      regulator.setpoint = 50;        // сообщаем регулятору температуру, которую он должен поддерживать
      // в процессе работы можно менять коэффициенты
      regulator.Kp = 5.2;
      regulator.Ki += 0.5;
      regulator.Kd = 0;

    menuSize = sizeof(menuPGM) / sizeof(menuPGM[0]);
    // for (uint8_t i = 0; i < menuSize; i++)
    // {
    //   if ( subMenuM.level < menuPGM[i].level)
    //      subMenuM.level = menuPGM[i].level;
    // }

    // Для первоначального обновления меню
    subMenuM.levelUpdate = DOWN;
    subMenuM.pointerUpdate = 1;
    subMenuM.parentID = 0;
    // level = 0;
    subMenuM.position = 0;
    memset(subMenuM.membersID, 0, sizeof(subMenuM.membersID) / sizeof(subMenuM.membersID[0]));
    Serial.println("menuSize: " + String(menuSize));
}

void loop()
{
    if(flagISR) {
            
        // if(isr_1) Serial.println("-----------> isr_1");
        // if(isr_2) Serial.println("-----------> isr_2");
        // if(isr_3) Serial.println("-----------> isr_3");

        intsFound = flagISR = 0;
        flagISR = 0;
        isr_1 = isr_2 = isr_3 = 0;
    }

    testPWM++;
    // analogWrite(DIMMER_PIN, testPWM);
    dimmer = map(testPWM, 0, 255, 500, 9300);
    analogWrite(buzzerPin, testPWM);
    analogWrite(FAN, testPWM);
    Serial.println("testTIMER_COUNT: " + String(testTIMER_COUNT) + "\ttestPWM: " + String(testPWM) + "\tdimmer: " + String(dimmer));


    int tmpTemp = analogRead(NTC_PIN);
    if (tmpTemp == ADC_MIN || tmpTemp >= ADC_MAX)
    {
        state = NTC_ERROR;
    }

    if (!digitalRead(encBut) && (state == DRY || state == STORAGE))
    {
        state = MENU;
        digitalWrite(DIMMER_PIN, 0);
        // tone(buzzerPin, 500, 100);
        enc.resetStates();
        subMenuM.levelUpdate = UP;
        subMenuM.pointerUpdate = 1;
        subMenuM.parentID = 0;
        delay(300);
    }

    switch (state)
    {
    case NTC_ERROR:
        dimmer = 0;
        digitalWrite(DIMMER_PIN, 0);
        detachInterrupt(INT_NUM);
        analogWrite(FAN, 255);
        do
        {
            oled.drawUTF8(32, 16, "ОШИБКА");
            oled.drawUTF8(32, 32, "ПРОВЕРЬ");
            oled.drawUTF8(26, 48, "ТЕРМИСТОР");
            oled.drawUTF8(0, 64, "");
            // oled.drawButtonUTF8(0, 32 - 1, U8G2_BTN_INV, 128, 1, 1, "");

        } while (oled.nextPage());
        while (1)
        {
            // tone(buzzerPin, 500, 500);
            delay(500);
            // tone(buzzerPin, 1000, 500);
        }
        break;
    // case OFF:
    //     /* code */
    //     break;
    case ON:
        break;
    case MENU:
        enc.tick();
        encoderSate(&controls);
        controlsHandler(menuPGM, menuVal, &controls, &subMenuM);
        if (subMenuM.levelUpdate)
        {
            submenuHandler(menuPGM, menuSize, &subMenuM);
            screen(&subMenuM); //!! добавить subMenuM.pointerUpdate  в submenuHandler
                               // submenuPrint();
        }
        if (subMenuM.pointerUpdate)
        {
            screen(&subMenuM);
            // submenuPrint();
        }
        dispalyPrint(&subMenuM);
        break;
    case DRY:

        // sprintf(bmePresureChar, "P: %04d mm", bme.readPressure() / 100.0F);
        // sprintf(bmeAltitude, "A: %04d m", bme.readAltitude(SEALEVELPRESSURE_HPA));
        // Serial.println("DRY");
        if (iDryer.getData(menuVal[3]))
        {
            dispalyPrintMode();
        }
        else
        {
            ERROR_COUNTER++;
            if (ERROR_COUNTER > MAX_ERROR) state = NTC_ERROR;
        }
        // regulator.setpoint = iDryer.temperature;
        // regulator.input = iDryer.data.ntcTemp;
        // dimmer = map(regulator.getResultTimer(), 0, 255, 500, 9300);
        if (millis() - startDry > menuVal[3]) //!!НАПИСАТЬ ИМЕНА
        {
            // Serial.println("DRY - > STORAGE");
            state = STORAGE; // автоматический переход в режим хранения
        }
        break;
    case STORAGE:
        // Serial.println("STORAGE");
        if (iDryer.getData(menuVal[3]))
        {
            dispalyPrintMode();
        }
        else
        {
            ERROR_COUNTER++;
            if (ERROR_COUNTER > MAX_ERROR) state = NTC_ERROR;
        }
        // regulator.setpoint = iDryer.temperature;
        // regulator.input = iDryer.data.ntcTemp;
        // dimmer = map(regulator.getResultTimer(), 0, 255, 500, 9300);
        break;
    case AUTOPID:
        while (pid_itr < 7) // AUTOPID
        {
            // направление, начальный сигнал, конечный, период плато, точность, время стабилизации, период итерации
            tuner.setParameters(NORMAL, 0, 80, 6000, 0.05, 500);

            // tuner.setInput(ntc.getTempAverage());
            tuner.setInput( ntc.analog2temp());
            tuner.compute();
            dimmer = map(tuner.getOutput(), 0, 255, 500, 9300);
            if (tuner.getState() != 7 && pid_itr != tuner.getState())
            {
                pid_itr = tuner.getState();
                // sprintf(str1, "PID ATOTUNE");
                // sprintf(str2, "Kp:  %.2f", tuner.getPID_p());
                // sprintf(str3, "Ki:  %.2f", tuner.getPID_i());
                // sprintf(str4, "Kd:  %.2f", tuner.getPID_d());
                // // dispalyPrint4(str1, str2, str3, str4);
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
                // dispalyPrint4("PID", "IS", "COMPUTE", "AND SAVE");
                // tone(buzzerPin, 500, 100);
                delay(100);
                // tone(buzzerPin, 500, 1000);
                // sprintf(str1, "CURRENT PID");
                // sprintf(str2, "Kp:  %.2f", tuner.getPID_p());
                // sprintf(str3, "Ki:  %.2f", tuner.getPID_i());
                // sprintf(str4, "Kd:  %.2f", tuner.getPID_d());
                // // dispalyPrint4(str1, str2, str3, str4);
                state = OFF;
            }
        }
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
        Serial.println("\t\t  enc.isClick()");
    }
    if (enc.isHolded())
    {
        control->hold = 1;
        Serial.print("\t\t  enc.isHolded()");
    }
    if (enc.isRelease())
    {
        enc.resetStates();
        Serial.print(" enc.isRelease() control->hold = 1;");
    }
    if (enc.isLeft())
    {
        control->left = 1;
        Serial.print("enc.isLeft()");
    }
    if (enc.isRight())
    {
        control->right = 1;
        Serial.print("enc.isRight()");
    }
}

void controlsHandler(const menuS constMenu[], uint16_t editableMenu[], struct control *encoder, struct subMenu *subMenu)
{
    if (encoder->ok)
    {
        encoder->ok = false;
        // Serial.print("encoder->ok");
        if (!pgm_read_word(&constMenu[subMenu->membersID[subMenu->position]].min) &&
            !pgm_read_word(&constMenu[subMenu->membersID[subMenu->position]].max) &&
            !pgm_read_byte(&constMenu[subMenu->membersID[subMenu->position]].action)) // меню
        {
            subMenu->level + 1 > subMenu->levelMax ? subMenu->level : subMenu->level++;
            subMenu->levelUpdate = DOWN;
        }
        else
        {
            if (pgm_read_byte(&constMenu[subMenu->membersID[subMenu->linesToScreen[subMenu->pointerPos]]].action)) // подменю с функцией
            {

                // Serial.print("subMenuM.membersID[subMenuM.linesToScreen[subMenuM.pointerPos]]: ");
                // Serial.println(subMenu->membersID[subMenu->linesToScreen[subMenu->pointerPos]]);
                // TODO Проверил, должно запускаться
                // constMenu[subMenu->membersID[subMenu->linesToScreen[subMenu->pointerPos]]].action();
                menuPGM[subMenu->membersID[subMenu->linesToScreen[subMenu->pointerPos]]].action();
                // break; // запускаем функцию
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
        // Serial.println(subMenu->parentID);
        // delay(800);
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
    // Serial.println("");
    for (uint8_t i = 0; i < menuSize; i++)
    {
        if (pgm_read_byte(&constMenu[i].parentID) == subMenu->parentID && pgm_read_byte(&constMenu[i].id) != 0)
        {
            subMenu->membersID[iCounter] = i;
            // Serial.println("subMenu->members: " + String(printMenuItem(&constMenu[subMenu->membersID[iCounter]].text)));
            // delay(20);
            if (i == posTmp)
            {
                Serial.println(String(i) + " : " + String(printMenuItem(&constMenu[i].text)) + "\tsubmenuHandler subMenu->position" + String(subMenu->position));
                subMenu->position = iCounter;
                subMenu->pointerUpdate = 1;
            }
            iCounter++;
        }
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
