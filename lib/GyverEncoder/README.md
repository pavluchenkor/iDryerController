[![Foo](https://img.shields.io/badge/Version-4.10-brightgreen.svg?style=flat-square)](#versions)
[![Foo](https://img.shields.io/badge/Website-AlexGyver.ru-blue.svg?style=flat-square)](https://alexgyver.ru/)
[![Foo](https://img.shields.io/badge/%E2%82%BD$%E2%82%AC%20%D0%9D%D0%B0%20%D0%BF%D0%B8%D0%B2%D0%BE-%D1%81%20%D1%80%D1%8B%D0%B1%D0%BA%D0%BE%D0%B9-orange.svg?style=flat-square)](https://alexgyver.ru/support_alex/)

## ВНИМАНИЕ, БИБЛИОТЕКА УСТАРЕЛА!
### ИСПОЛЬЗУЙ БИБЛИОТЕКУ [EncButton](https://github.com/GyverLibs/EncButton)
Она гораздо легче, имеет больше возможностей и использует меньше процессорного времени!

```






```

# GyverEncoder
Библиотека для расширенной работы с энкодером  
- Отработка поворота энкодера
- Отработка "нажатого поворота"	
- Отработка "быстрого поворота"
- Несколько алгоритмов опроса энкодера
- Выбор подтяжки подключения энкодера
- Работа с двумя типами экнодеров
- Работа с внешним энкодером (через расширитель пинов и т.п.)
- Отработка нажатия/удержания кнопки с антидребезгом

### Совместимость
Совместима со всеми Arduino платформами (используются Arduino-функции)

### Документация
К библиотеке есть [расширенная документация](https://alexgyver.ru/encoder/)

## Содержание
- [Установка](#install)
- [Инициализация](#init)
- [Использование](#usage)
- [Пример](#example)
- [Версии](#versions)
- [Баги и обратная связь](#feedback)

<a id="install"></a>
## Установка
- Библиотеку можно найти по названию **GyverEncoder** и установить через менеджер библиотек в:
    - Arduino IDE
    - Arduino IDE v2
    - PlatformIO
- [Скачать библиотеку](https://github.com/GyverLibs/GyverEncoder/archive/refs/heads/main.zip) .zip архивом для ручной установки:
    - Распаковать и положить в *C:\Program Files (x86)\Arduino\libraries* (Windows x64)
    - Распаковать и положить в *C:\Program Files\Arduino\libraries* (Windows x32)
    - Распаковать и положить в *Документы/Arduino/libraries/*
    - (Arduino IDE) автоматическая установка из .zip: *Скетч/Подключить библиотеку/Добавить .ZIP библиотеку…* и указать скачанный архив
- Читай более подробную инструкцию по установке библиотек [здесь](https://alexgyver.ru/arduino-first/#%D0%A3%D1%81%D1%82%D0%B0%D0%BD%D0%BE%D0%B2%D0%BA%D0%B0_%D0%B1%D0%B8%D0%B1%D0%BB%D0%B8%D0%BE%D1%82%D0%B5%D0%BA)

<a id="init"></a>
## Инициализация
```cpp
Encoder enc;                                        // не привязан к пину
Encoder enc(пин CLK, пин DT);                       // энкодер без кнопки (ускоренный опрос)
Encoder enc(пин CLK, пин DT, пин SW);               // энкодер с кнопкой
Encoder enc(пин CLK, пин DT, пин SW, тип);          // энкодер с кнопкой и указанием типа
Encoder enc(пин CLK, пин DT, ENC_NO_BUTTON, тип);   // энкодер без кнопкой и с указанием типа
```

<a id="usage"></a>
## Использование
```cpp
void tick();                             // опрос энкодера, нужно вызывать постоянно или в прерывании
void setType(boolean type);              // TYPE1 / TYPE2 - тип энкодера TYPE1 одношаговый, TYPE2 двухшаговый. Если ваш энкодер работает странно, смените тип
void setTickMode(boolean tickMode);      // MANUAL / AUTO - ручной или автоматический опрос энкодера функцией tick(). (по умолчанию ручной)
void setDirection(boolean direction);    // NORM / REVERSE - направление вращения энкодера
void setFastTimeout(int timeout);        // установка таймаута быстрого поворота
void setPinMode(bool mode);              // тип подключения энкодера, подтяжка HIGH_PULL (внутренняя) или LOW_PULL (внешняя на GND)
void setBtnPinMode(bool mode);           // тип подключения кнопки, подтяжка HIGH_PULL (внутренняя) или LOW_PULL (внешняя на GND)
 
boolean isTurn();                        // возвращает true при любом повороте, сама сбрасывается в false
boolean isRight();                       // возвращает true при повороте направо, сама сбрасывается в false
boolean isLeft();                        // возвращает true при повороте налево, сама сбрасывается в false
boolean isRightH();                      // возвращает true при удержании кнопки и повороте направо, сама сбрасывается в false
boolean isLeftH();                       // возвращает true при удержании кнопки и повороте налево, сама сбрасывается в false
boolean isFastR();                       // возвращает true при быстром повороте
boolean isFastL();                       // возвращает true при быстром повороте
 
boolean isPress();                       // возвращает true при нажатии кнопки, сама сбрасывается в false
boolean isRelease();                     // возвращает true при отпускании кнопки, сама сбрасывается в false
boolean isClick();                       // возвращает true при нажатии и отпускании кнопки, сама сбрасывается в false
boolean isHolded();                      // возвращает true при удержании кнопки, сама сбрасывается в false
boolean isHold();                        // возвращает true при удержании кнопки, НЕ СБРАСЫВАЕТСЯ
boolean isSingle();                      // возвращает true при одиночном клике (после таймаута), сама сбрасывается в false
boolean isDouble();                      // возвращает true при двойном клике, сама сбрасывается в false
void resetStates();                      // сбрасывает все is-флаги
```

<a id="example"></a>
## Пример
Остальные примеры смотри в **examples**!
```cpp
#define CLK 2
#define DT 3
#define SW 4

#include "GyverEncoder.h"
Encoder enc1(CLK, DT, SW);  // для работы c кнопкой

void setup() {
  Serial.begin(9600);
  enc1.setType(TYPE2);
}

void loop() {
	// обязательная функция отработки. Должна постоянно опрашиваться
  enc1.tick();
  
  if (enc1.isTurn()) {     // если был совершён поворот (индикатор поворота в любую сторону)
    // ваш код
  }
  
  if (enc1.isRight()) Serial.println("Right");         // если был поворот
  if (enc1.isLeft()) Serial.println("Left");
  
  if (enc1.isRightH()) Serial.println("Right holded"); // если было удержание + поворот
  if (enc1.isLeftH()) Serial.println("Left holded");
  
  //if (enc1.isPress()) Serial.println("Press");         // нажатие на кнопку (+ дебаунс)
  //if (enc1.isRelease()) Serial.println("Release");     // то же самое, что isClick
  
  if (enc1.isClick()) Serial.println("Click");         // одиночный клик
  if (enc1.isSingle()) Serial.println("Single");       // одиночный клик (с таймаутом для двойного)
  if (enc1.isDouble()) Serial.println("Double");       // двойной клик
  
  
  if (enc1.isHolded()) Serial.println("Holded");       // если была удержана и энк не поворачивался
  //if (enc1.isHold()) Serial.println("Hold");         // возвращает состояние кнопки
}
```

<a id="versions"></a>
## Версии
- v3.6 от 16.09.2019	- Возвращены дефайны настроек
- v4.0 от 13.11.2019
        - Оптимизирован код
        - Исправлены баги
        - Добавлены другие алгоритмы опроса
        - Добавлена возможность полностью убрать кнопку (экономия памяти)
        - Добавлена возможность подключения внешнего энкодера
        - Добавлена настройка подтяжки пинов        
- v4.1: Исправлено изменение подтяжек        
- v4.2
        - Добавлена поддержка TYPE1 для алгоритма PRECISE_ALGORITHM
        - Добавлена отработка двойного клика: isSingle / isDouble        
- v4.3: Исправлено ложное isSingle		
- v4.4: Добавлен метод resetStates, сбрасывает все is-флаги и счётчики
- v4.5: Улучшен алгоритм BINARY_ALGORITHM (спасибо Ярославу Курусу)
- v4.6: BINARY_ALGORITHM пофикшен для TYPE1, добавлена isReleaseHold
- v4.7: Исправлен случайный нажатый поворот в BINARY_ALGORITHM
- v4.8: увеличена производительность для AVR Arduino
- v4.9: быстрый поворот отключен если кнопка удерживается
- v4.10: починил setDirection()

<a id="feedback"></a>
## Баги и обратная связь
При нахождении багов создавайте **Issue**, а лучше сразу пишите на почту [alex@alexgyver.ru](mailto:alex@alexgyver.ru)  
Библиотека открыта для доработки и ваших **Pull Request**'ов!
