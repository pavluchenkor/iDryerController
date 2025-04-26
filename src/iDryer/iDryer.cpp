#include "iDryer.h"

bool Data::operator!=(const Data &other) const
{
  return timestamp != other.timestamp || int(ntcTemp) != int(other.ntcTemp) || int(airTemp) != int(other.airTemp) || int(airHumidity) != int(other.airHumidity);
}

iDryer::iDryer(thermistor &ntc) : ntc(ntc)
{
}

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

  if (data.airTemp <= MIN_CALIB_TEMP)
  {
    data.airTempCorrected = data.airTemp;
  }
  else
  {
    data.airTempCorrected = math::map_to_range(data.airTemp, MIN_CALIB_TEMP, MAX_CALIB_TEMP, REAL_CALIB_TEMP_MIN, REAL_CALIB_TEMP_MAX);
  }

  if (data != oldData && data.timestamp - screenTime > SCREEN_UPADATE_TIME)
  {
    screenTime = data.timestamp;
    data.flagScreenUpdate = true;
    oldData = data;
  }
  else
  {
    data.flagScreenUpdate = false;
  }

  if (uint8_t(ceil(data.airTempCorrected)) >= data.setTemp && !data.flagTimeCounter)
    data.flagTimeCounter = true;

  if (data.ntcTemp < TMP_MIN)
    return false;

  if (data.ntcTemp > TMP_MAX + TMP_SAFETY_THRESHOLD)
    return false;

  if (data.airTempCorrected < TMP_MIN)
    return false;

  if (data.airTempCorrected > TMP_MAX + TMP_SAFETY_THRESHOLD)
    return false;

  return true;
}
