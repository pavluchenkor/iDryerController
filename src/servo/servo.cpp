#include "servo.h"

servo::servo(uint8_t _srvPin, uint16_t _closedTime, uint16_t _openTime, uint16_t _angle)
{
  pin = _srvPin;
  closedTime = _closedTime;
  openTime = _openTime;
  angle = _angle * angleMultiplier;
}

void servo::set(uint16_t _closedTime, uint16_t _openTime, uint16_t _angle)
{
  closedTime = _closedTime;
  openTime = _openTime;
  angle = _angle * angleMultiplier;
}

void servo::updateServo()
{
  // if (setAngle(currentAngle, angle))
  if (setAngle())
  {
    if (changeState)
    {

      if (currentAngle == closedAngle)
      {
        state = CLOSED;
      }
      else
      {
        state = OPEN;
      }
      changeState = 0;
    }
  }
  else
  {
    if (servoPin)
    {
      PORTD &= ~(1 << pin);
      Timer1.setPeriod(SERVO_PERIOD_MS * 1000 - pulseWidth);
      servoPin = LOW;
    }
    else
    {
      pulseWidth = map(currentAngle, 0, 180 * angleMultiplier, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
      PORTD |= (1 << pin);
      Timer1.setPeriod(pulseWidth);
      servoPin = HIGH;
    }
  }
}

bool servo::setAngle()
{
  if (currentAngle != angle)
  {
    if (state != MOVE)
    {
      prevState = state;
      state = MOVE;
      PORTD &= ~(1 << pin);
      Timer1.enableISR(CHANNEL_A);
    }
    if (currentAngle < angle)
    {
      currentAngle++;
    }
    else
    {
      currentAngle--;
    }
    return 0;
  }
  else
  {
    return 1;
  }
}

void servo::close()
{
  state = OPEN;
  currentAngle = 130;
  changeState = 1;
  angle = closedAngle;
  updateServo();
}

void servo::toggle()
{
  if (state == CLOSED)
  {
    changeState = 1;
    angle = closedAngle + angle;
  }
  if (state == OPEN)
  {
    changeState = 1;
    angle = closedAngle;
  }
  updateServo();
}

void servo::check()
{
  if (state == CLOSED)
  {
    if (servoOldTime < millis() && openTime)
    {
      toggle();
      servoOldTime = millis() + (unsigned long)openTime * 1000UL * 60UL;
    }
    updateServo();
  }
  if (state == OPEN)
  {
    if (servoOldTime < millis() && closedTime)
    {
      toggle();
      servoOldTime = millis() + (unsigned long)closedTime * 1000UL * 60UL;
    }
    updateServo();
  }
}