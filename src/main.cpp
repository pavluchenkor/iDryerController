//!* OLED https://startingelectronics.org/tutorials/arduino/modules/OLED-128x32-I2C-display/
//!* BME280 https://randomnerdtutorials.com/bme280-sensor-arduino-pressure-temperature-humidity/
//!* https://github.com/AlexGyver/tutorials/tree/master/triac/smooth_1ch
//!!https://github.com/AlexGyver/AC_Dimmer  схемы и всякое такое

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <U8g2lib.h>
#include <GyverEncoder.h>
#include <GyverTimers.h>  // библиотека таймера
#include <GyverNTC.h>           
//Энкодер
#define DT 7						   // Pin  Detect
#define CLK 6						   // Pin  Clockwise
#define encBut 5					   // Pin encoder Button
//Энкодер

//Димер
#define ZERO_PIN 2    // пин детектора нуля
#define INT_NUM 0     // соответствующий ему номер прерывания
#define DIMMER_PIN 4  // управляющий пин симистора
//Димер

#define NTC_PIN 0
#define SEALEVELPRESSURE_HPA (1013.25) // оценивает высоту в метрах на основе давления на уровне моря

const PROGMEM char arButt[36] = {'Q', 'W', 'E', 'R', 'T', 'Y', 'U', 'I', 'O', 'P', 'A', 'S', 'D', 'F', 'G', 'H', 'J', 'K', 'L', 'Z', 'X', 'C', 'V', 'B', 'N', 'M', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0'};
const PROGMEM char arButtSmall[36] = {'q', 'w', 'e', 'r', 't', 'y', 'u', 'i', 'o', 'p', 'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', 'z', 'x', 'c', 'v', 'b', 'n', 'm', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0'};

U8G2_SSD1306_128X64_NONAME_1_HW_I2C oled(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
Adafruit_BME280 bme;
Encoder enc(CLK, DT, encBut);	
GyverNTC ntc(NTC_PIN, 10000, 3950);

int dimmer;  // переменная диммера

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

// прерывание детектора нуля
void isr() {
  static int lastDim;
  digitalWrite(DIMMER_PIN, 0);  // выключаем симистор
  // если значение изменилось, устанавливаем новый период
  // если нет, то просто перезапускаем со старым
  if (lastDim != dimmer) Timer2.setPeriod(lastDim = dimmer);
  else Timer2.restart();
}

// прерывание таймера
ISR(TIMER2_A) {
  digitalWrite(DIMMER_PIN, 1);  // включаем симистор
  Timer2.stop();                // останавливаем таймер
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

void setup()
{
	//Диммер
	pinMode(ZERO_PIN, INPUT_PULLUP);
	pinMode(DIMMER_PIN, OUTPUT);

	pinMode(DT, INPUT_PULLUP);
  	pinMode(CLK, INPUT_PULLUP);
  	pinMode(encBut, INPUT);
	
	attachInterrupt(INT_NUM, isr, RISING); // для самодельной схемы ставь FALLING
	Timer2.enableISR();
	//Диммер

	enc.setType(TYPE2);
	attachInterrupt(0, isrCLK, CHANGE); // прерывание на 2 пине! CLK у энка
	attachInterrupt(1, isrDT, CHANGE);	// прерывание на 3 пине! DT у энка
	Serial.begin(9600);

	if (!bme.begin(0x76))
	{
		Serial.println("Could not find a valid BME280 sensor, check wiring!");
		while (1)
			;
	}
	oled.begin();
	oled.setFlipMode(1);
	oled.setContrast(0);
	delay(200);
	dispalyPrint4("-", "-", "-", "-"); //!!!! НЕ УДАЛЯТЬ ИНАЧЕ СТАРТУЕТ КРИВО, ВИДИМО НУЖНО СНАЧАЛА ПНУТЬ ЭКРАН
	dispalyPrint4("SUPER", "PUPER", "SUSHILKA", "BY ENGENNER");
	delay(3000);
}

void loop()
{
	enc.tick();

	char bmeTemp[3];
	char ntcTemp[3];
	char bmeHumidity[3];
	char bmePresure[4];
	char bmeAltitude[4];
	sprintf(bmeTemp, "t:  %03d C", bme.readTemperature());
	sprintf(bmeHumidity, "H:  %03d %", bme.readHumidity());
	sprintf(bmePresure, "P: %04d mm", bme.readPressure() / 100.0F);
	sprintf(ntcTemp, "P: %04d mm", round(ntc.getTempAverage()));
	dispalyPrint4(bmeTemp, bmeHumidity, bmePresure, bmeAltitude);
	// sprintf(bmeAltitude, "A: %04d m", bme.readAltitude(SEALEVELPRESSURE_HPA));

	// задаём значение 500-9300, где 500 максимум мощности, 9300 минимум!!!
	// и 500-7600 для 60 Гц в сети
	dimmer = map(analogRead(A0), 0, 1024, 500, 9300); //!! для чтения потенциаометра, потом заменить на нужный показатель в процентах
	delay(100); // в реальном коде задержек быть не должно
 
}