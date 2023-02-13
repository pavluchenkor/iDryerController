// Используем аппаратный таймер вместо софтверного millis()/micros() БЕЗ прерываний
// Подойдет для быстрых событий с высокой точностью!

#include <GyverTimers.h>

void setup() {
  Serial.begin(9600);      // Для примера откроем порт

  Timer1.setFrequency(3);  // Задать частоту в целых числах
  // С этого момента таймер уже перенастроен и гоняет с выбранной частотой / периодом
}

void loop() {
  static uint32_t timer = millis();
  if (millis() - timer >= 1000) {
    timer = millis();
    Serial.println("Software timer!");
  }

  if (Timer1.ready(CHANNEL_A)) { // Если прерывание канала А таймера 1 готово (но мы его не включили)
    Timer1.stop();               // Тормозим таймер (при желании)
    Serial.println("Hardware timer!");
    Timer1.restart();            // Перезапускаем таймер (раз затормозили)
  }
}
