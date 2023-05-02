// пример диммера на симисторе с внешним детектором нуля
// или готовый китайский модуль https://ali.ski/yGU73N
// используется библиотека GyverTimers (минимум версия 1.5)
// по аналогии можно сделать диммер на любом таймере ардуино
// в том числе любом из 6-ти таймеров для Mega
// вызов setPeriod оптимизирован
#include <Arduino.h>
#define ZERO_PIN 2       // пин детектора нуля
#define INT_NUM 0        // соответствующий ему номер прерывания
#define DIMMER_PIN 5     // управляющий пин симистора
#define FAN 6
#include <GyverTimers.h> // библиотека таймера
int dimmer;              // переменная диммера
int isrCounter;
int timerCounter;
unsigned long time = 0;
unsigned long time1 = 0;
unsigned long time2 = 0;


// void timerInit()
// {
//   cli();
//   TCCR0B = 0; 
//   OCR2A = 9999;
//   TCCR2A = (1 << WGM21);
//   TCCR2B = (1 << CS21);
//   TIMSK2 = (1 << OCIE2A);
//   sei();
// }

// void timerStart(uint32_t time_ms)
// {
//   OCR2A = (F_CPU / (8 * 1000UL)) * time_ms;
//   TCCR2B |= (1 << CS21);
// }

// void timerStop()
// {
// TCCR2B = 0;
// }

// uint32_t period(long time_ms, uint8_t DIV)
// {
//   return (F_CPU / (DIV * 1000UL)) * time_ms;;
// }


void isr()
{
  // digitalWrite(DIMMER_PIN, 0);
  Timer2.setPeriod(dimmer);
  // timerStop();
  // Timer2.restart();
}

// // ISR(Timer2_COMPA_vect)
// // {
// //   time2 = micros() - time1;
// //   digitalWrite(DIMMER_PIN, 1); // включаем симистор
// //   digitalWrite(DIMMER_PIN, 0);
// //   timerStop();
// // }
// // // прерывание таймера
ISR(TIMER2_A)
{
  digitalWrite(DIMMER_PIN, 1); // включаем симистор
  digitalWrite(DIMMER_PIN, 0);
  Timer2.stop();  
}


void setup()
{
  Serial.begin(9600);
  pinMode(ZERO_PIN, INPUT_PULLUP);
  pinMode(DIMMER_PIN, OUTPUT);
  pinMode(FAN, OUTPUT);
  attachInterrupt(INT_NUM, isr, RISING); //FALLING
  Timer2.enableISR();
}

void loop()
{
analogWrite(FAN, 250);
Serial.print(timerCounter);
Serial.print(" ");
Serial.print(dimmer);
Serial.print(" ");
Serial.print(time1);
Serial.print(" ");
Serial.println(time2);
dimmer > 9500 ? dimmer = 8500 : dimmer+=10;

}


