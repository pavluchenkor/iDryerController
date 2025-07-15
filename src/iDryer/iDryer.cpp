#include "iDryer.h"

#ifdef SENSOR_BME280
iDryer::iDryer(thermistor &ntc, GyverBME280 &bme) : ntc(ntc), bme(bme)
{
}
#else
iDryer::iDryer(thermistor &ntc, SHT31 &sht) : ntc(ntc), sht(sht)
{
}
#endif

bool iDryer::getData()
{
  data.timestamp = millis();
  data.ntcTemp = (ntc.analog2temp() + data.ntcTemp) / 2.0f;

#ifdef SENSOR_SHT31
  if (sht.dataReady())
  {
    sht.read();
    data.airTemp = (sht.getTemperature() + data.airTemp) / 2.0f;
    data.airHumidity = (sht.getHumidity() + data.airHumidity) / 2.0f;
  }
#endif

#ifdef SENSOR_BME280
  data.airTemp = (bme.readTemperature() + data.airTemp) / 2.0f;
  data.airHumidity = (bme.readHumidity() + data.airHumidity) / 2.0f;
#endif

  data.airTempCorrected = math::map_to_range_with_clamp(data.airTemp, MIN_CALIB_TEMP, MAX_CALIB_TEMP, REAL_CALIB_TEMP_MIN, REAL_CALIB_TEMP_MAX);
  data.flagScreenUpdate = false;

  if (data.timestamp - lastScreenUpdateTimestamp > SCREEN_UPADATE_TIME)
  {
    lastScreenUpdateTimestamp = data.timestamp;
    data.flagScreenUpdate = true;
  }

  if (!data.flagTimeCounter && (uint8_t(round(data.airTempCorrected)) >= data.setTemp - DRY_START_THRESHOLD))
  {
    data.flagTimeCounter = true;
  }

  if (data.ntcTemp < TMP_MIN)
  {
    return false;
  }

  if (data.ntcTemp > TMP_MAX + TMP_SAFETY_THRESHOLD)
  {
    return false;
  }

  if (data.airTempCorrected < TMP_MIN)
  {
    return false;
  }

  if (data.airTempCorrected > TMP_MAX + TMP_SAFETY_THRESHOLD)
  {
    return false;
  }

  return true;
}
