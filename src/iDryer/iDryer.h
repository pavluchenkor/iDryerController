#ifndef IDRYER_H
#define IDRYER_H

#include <Arduino.h>
#include <GyverTimers.h>
#include "Configuration.h"
#include "math/math_extensions.h"
#include "thermistor/thermistor.h"
#include "GyverBME280.h"
#include "SHT31.h"

enum State
{
  OFF,
  ON,
  MENU,
  DRY,
  STORAGE,
  AUTOPID,
  NTC_ERROR,
};

struct Data
{
  unsigned long timestamp = 0; // timestamp in ms
  float ntcTemp = 0;
  float airTemp = 0;
  float airTempCorrected = 0;
  float airHumidity = 0;
  bool optimalConditionsReachedFlag = false;
  unsigned long startTime = 0;
  uint8_t setTemp = 0;
  uint8_t setHumidity = 0;
  uint16_t setTime = 0;
  bool flag = false;
  bool flagScreenUpdate = false;
  bool flagTimeCounter = false;
  uint8_t setFan = 0;
  float Kp = 0;
  float Ki = 0;
  float Kd = 0;
  float Kf = 0;
  float minDeltaTime = 0;
  uint8_t deltaT = 0;
};

class iDryer
{
public:
  Data data;
  unsigned long lastScreenUpdateTimestamp = 0;
  thermistor &ntc;

#ifdef SENSOR_BME280
  GyverBME280 &bme;
#else
  SHT31 &sht;
#endif

#ifdef SENSOR_BME280
  iDryer(thermistor &ntc, GyverBME280 &bme);
#else
  iDryer(thermistor &ntc, SHT31 &sht);
#endif

  bool getData();
};

#endif // IDRYER_H
