#ifndef IDRYER_H
#define IDRYER_H

#include <Arduino.h>
#include <GyverTimers.h>
#include "Configuration.h"
#include "math/math_extensions.h"
#include "thermistor/thermistor.h"

struct Data
{
  unsigned long timestamp = 0;
  float ntcTemp = 0;
  float airTemp = 0;
  float airTempCorrected = 0;
  float airHumidity = 0;
  bool optimalConditionsReachedFlag = false;
  unsigned long startTime = 0;
  unsigned long errorTime28 = 0;
  uint8_t setTemp = 0;
  uint8_t setHumidity = 0;
  uint16_t setTime = 0;
  bool flag = false;
  bool flagScreenUpdate = false;
  bool flagTimeCounter = false;
  uint8_t setFan = 0;
  float Kp = 0.0f;
  float Ki = 0.0f;
  float Kd = 0.0f;
  uint16_t sampleTime = 0;
  uint8_t deltaT = 0;

  bool operator!=(const Data &other) const;
};

class iDryer
{
public:
  Data data;
  Data oldData;
  unsigned long screenTime = 0;
  thermistor &ntc;

  iDryer(thermistor &ntc);

  bool getData();
};

#endif // IDRYER_H
