#ifndef BUZZER_H
#define BUZZER_H

#include <Arduino.h>

class BuzzerController
{
private:
  const uint8_t buzzerPin;
  bool isBuzzing;
  unsigned long endTime;

public:
  BuzzerController(uint8_t buzzerPin);

  void buzz(uint16_t duration);

  void update();

  void stop();
};

#endif // BUZZER_H
