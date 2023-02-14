![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)
![author](https://img.shields.io/badge/author-AlexGyver-informational.svg)
# GyverBME280
Лёгкая библиотека для работы с BME280 по I2C для Arduino

### Совместимость
Совместима со всеми Arduino платформами (используются Arduino-функции)

## Содержание
- [Установка](#install)
- [Инициализация](#init)
- [Использование](#usage)
- [Пример](#example)
- [Версии](#versions)
- [Баги и обратная связь](#feedback)

<a id="install"></a>
## Установка
- Библиотеку можно найти по названию **GyverBME280** и установить через менеджер библиотек в:
    - Arduino IDE
    - Arduino IDE v2
    - PlatformIO
- [Скачать библиотеку](https://github.com/GyverLibs/GyverBME280/archive/refs/heads/main.zip) .zip архивом для ручной установки:
    - Распаковать и положить в *C:\Program Files (x86)\Arduino\libraries* (Windows x64)
    - Распаковать и положить в *C:\Program Files\Arduino\libraries* (Windows x32)
    - Распаковать и положить в *Документы/Arduino/libraries/*
    - (Arduino IDE) автоматическая установка из .zip: *Скетч/Подключить библиотеку/Добавить .ZIP библиотеку…* и указать скачанный архив
- Читай более подробную инструкцию по установке библиотек [здесь](https://alexgyver.ru/arduino-first/#%D0%A3%D1%81%D1%82%D0%B0%D0%BD%D0%BE%D0%B2%D0%BA%D0%B0_%D0%B1%D0%B8%D0%B1%D0%BB%D0%B8%D0%BE%D1%82%D0%B5%D0%BA)

<a id="init"></a>
## Инициализация
```cpp
GyverBME280 bme;
```

<a id="usage"></a>
## Использование
```cpp
bool begin(void);                           // запустить со стандартным адресом (0x76)
bool begin(uint8_t address);                // запустить с указанием адреса
bool isMeasuring(void);                     // возвращает true, пока идёт измерение
float readPressure(void);                   // прочитать давление в Па
float readHumidity(void);                   // Прочитать влажность в %
void oneMeasurement(void);                  // Сделать одно измерение и уйти в сон
float readTemperature(void);                // Прочитать температуру в градусах С

void setMode(uint8_t mode);                 // установить режим работы
// режимы:
NORMAL_MODE
FORCED_MODE

void setFilter(uint8_t mode);               // изменить коэффициент фильтрации. Вызывать перед begin
// коэффициенты:
FILTER_DISABLE
FILTER_COEF_2
FILTER_COEF_4
FILTER_COEF_8
FILTER_COEF_16

void setStandbyTime(uint8_t mode);          // Изменить время между измерениями. Вызывать перед begin
// режимы:
STANDBY_500US
STANDBY_10MS
STANDBY_20MS
STANDBY_6250US
STANDBY_125MS
STANDBY_250MS
STANDBY_500MS
STANDBY_1000MS

void setHumOversampling(uint8_t mode);      // Настроить оверсэмплинг или отключить влажность. Вызывать перед begin
void setTempOversampling(uint8_t mode);     // Настроить оверсэмплинг или отключить температуру. Вызывать перед begin
void setPressOversampling(uint8_t mode);    // Настроить оверсэмплинг или отключить давление. Вызывать перед begin
// режимы:
MODULE_DISABLE
OVERSAMPLING_1
OVERSAMPLING_2
OVERSAMPLING_4
OVERSAMPLING_8
OVERSAMPLING_16
```

<a id="example"></a>
## Пример
Остальные примеры смотри в **examples**!
```cpp
/*
   Простой пример, демонстрирующий основные функции измерения температуры, давления и влажности
*/

#include <GyverBME280.h>                      // Подключение библиотеки
GyverBME280 bme;                              // Создание обьекта bme

void setup() {
  Serial.begin(9600);                         // Запуск последовательного порта
  bme.begin();                                // Если доп. настройки не нужны  - инициализируем датчик
}

void loop() {
  Serial.print("Temperature: ");
  Serial.print(bme.readTemperature());        // Выводим темперутуру в [*C]
  Serial.println(" *C");

  Serial.print("Humidity: ");
  Serial.print(bme.readHumidity());           // Выводим влажность в [%]
  Serial.println(" %");

  float pressure = bme.readPressure();        // Читаем давление в [Па]
  Serial.print("Pressure: ");
  Serial.print(pressure / 100.0F);            // Выводим давление в [гПа]
  Serial.print(" hPa , ");
  Serial.print(pressureToMmHg(pressure));     // Выводим давление в [мм рт. столба]
  Serial.println(" mm Hg");
  Serial.print("Altitide: ");
  Serial.print(pressureToAltitude(pressure)); // Выводим высоту в [м над ур. моря]
  Serial.println(" m");
  Serial.println("");
  delay(1000);
}
```

<a id="versions"></a>
## Версии
- v1.3 - исправлена ошибка при отриц. температуре
- v1.4 - разбил на h и cpp
- v1.5 - добавлена поддержка BMP280

<a id="feedback"></a>
## Баги и обратная связь
При нахождении багов создавайте **Issue**, а лучше сразу пишите на почту [alex@alexgyver.ru](mailto:alex@alexgyver.ru)  
Библиотека открыта для доработки и ваших **Pull Request**'ов!