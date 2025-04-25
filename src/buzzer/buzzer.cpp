#include "buzzer.h"

BuzzerController::BuzzerController(uint8_t buzzerPin)
    : buzzerPin(buzzerPin), isBuzzing(false), endTime(0)
{
}

void BuzzerController::buzz(uint16_t duration)
{
  if (!isBuzzing)
  {
      isBuzzing = true;
      endTime = millis() + duration;
      digitalWrite(buzzerPin, HIGH);
  }
}

void BuzzerController::update()
{
  if (isBuzzing && millis() >= endTime)
  {
      stop();
  }
}

void BuzzerController::stop()
{
  if (isBuzzing)
  {
      isBuzzing = false;
      digitalWrite(buzzerPin, LOW);
  }
}
