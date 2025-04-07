#ifndef SERVO_H
#define SERVO_H

#include <Arduino.h>
#include <GyverTimers.h>
#include "Configuration.h"

enum stateServo
{
  CLOSED,
  MOVE,
  OPEN,
};

class servo
{
public:
  uint8_t pin = 0;
  unsigned long servoOldTime = 0;

  uint16_t openTime = 0;
  uint16_t closedTime = 0;

  uint8_t angleMultiplier = 1;
  uint16_t closedAngle = 90 * angleMultiplier;
  uint16_t angle = 90;
  uint16_t currentAngle = 90;

  uint8_t changeState = 0;
  uint8_t servoPin = 0;
  uint16_t pulseWidth = 0;
  stateServo prevState = OPEN;
  stateServo state = CLOSED;

  servo(uint8_t _srvPin, uint16_t _closedTime, uint16_t _openTime, uint16_t _angle);

  void set(uint16_t _closedTime, uint16_t _openTime, uint16_t _angle);

  void updateServo();

  // bool setAngle(uint16_t &currentAngle, uint16_t &angle)
  bool setAngle();

  void close();

  void toggle();

  void check();
};

#endif // SERVO_H
