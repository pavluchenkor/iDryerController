#ifndef SERVO_H
#define SERVO_H

#include <Arduino.h>
#include <GyverTimers.h>
#include "Configuration.h"

enum ServoState
{
  UNKNOWN = 0,
  CLOSED,
  MOVE,
  OPEN,
};

class Servo
{
private:
  unsigned long _currentTimeMs = 0;
  unsigned long _prevLogTimeMs = 0;

  unsigned long _nextToggleTimeMs = 0;
  unsigned long _nextMoveTimeMs = 0;

  uint16_t _openedDuration = 0;
  uint16_t _closedDuration = 0;
  uint16_t _moveDuration = 0;

  uint16_t _closedAngle = 0;
  uint16_t _openedAngle = 0;
  uint16_t _targetAngle = 0;

  uint16_t _pulseWidth = 0;

  ServoState _state = UNKNOWN;

public:
  Servo();

  ServoState GetState() const;

  uint16_t GetPulseWidth() const;

  void Set(uint16_t closedDuration, uint16_t openedDuration, uint16_t openedAngle);

  void Update();

  void Toggle();

  void Autotoggle();

  void Open();
  
  void Close();
};

#endif // SERVO_H
