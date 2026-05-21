#include "servo.h"

Servo::Servo()
{
  _moveDuration = 1000; // Длительность изменения положения сервопривода в миллисекундах
}

ServoState Servo::GetState() const
{
  return _state;
}

uint16_t Servo::GetPulseWidth() const
{
  return _pulseWidth;
}

void Servo::Set(uint16_t closedDuration, uint16_t openedDuration, uint16_t openedAngle)
{
  _closedDuration = closedDuration;
  _openedDuration = openedDuration;
  _openedAngle = openedAngle;
}

void Servo::Update()
{
  _currentTimeMs = millis();

  if (_state == UNKNOWN)
  {
    Close();
  }

  if (_state == MOVE)
  {
    _pulseWidth = map(_targetAngle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);

    if (_nextMoveTimeMs < _currentTimeMs)
    {
      _state = (_targetAngle == _openedAngle) ? OPEN : CLOSED;
    }
  }

  if (_currentTimeMs - _prevLogTimeMs >= 100)
  {
    _prevLogTimeMs = _currentTimeMs;

#if KASYAK_FINDER && SERVO_LOGS
    Serial.print(" t: ");
    Serial.print(_currentTimeMs);
    Serial.print(" s: ");
    Serial.print(_state);
    Serial.print(" a: ");
    Serial.print(_targetAngle);
    Serial.print(" p: ");
    Serial.print(_pulseWidth);
    Serial.print(" nt: ");
    Serial.print(_nextToggleTimeMs);
    Serial.print(" nm: ");
    Serial.print(_nextMoveTimeMs);
    Serial.println();
    Serial.flush();
#endif
  }
}

void Servo::Toggle()
{
  if (_state == CLOSED)
  {
    Open();
  }
  if (_state == OPEN)
  {
    Close();
  }
}

void Servo::Autotoggle()
{
  if (_state == CLOSED && _nextToggleTimeMs < _currentTimeMs && _openedDuration > 0)
  {
    Toggle();
    _nextToggleTimeMs = _currentTimeMs + _openedDuration * 1000UL * 60UL;
  }

  if (_state == OPEN && _nextToggleTimeMs < _currentTimeMs && _closedDuration > 0)
  {
    Toggle();
    _nextToggleTimeMs = _currentTimeMs + _closedDuration * 1000UL * 60UL;
  }
}

void Servo::Open()
{
  _targetAngle = _openedAngle;
  _state = MOVE;
  _nextMoveTimeMs = millis() + _moveDuration;
}

void Servo::Close()
{
  _targetAngle = _closedAngle;
  _state = MOVE;
  _nextMoveTimeMs = millis() + _moveDuration;
}
