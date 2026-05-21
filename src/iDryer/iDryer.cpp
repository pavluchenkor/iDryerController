#include "iDryer.h"

iDryer::iDryer(thermistor &heaterTempSensor, AirTempSensor &airTempSensor) : _heaterTempSensor(heaterTempSensor), _airTempSensor(airTempSensor)
{
  airPid.SetProportionalGain(AIR_PID_KP);
  airPid.SetIntegralGain(AIR_PID_KI);
  airPid.SetDerivativeGain(AIR_PID_KD);
}

float iDryer::GetSetpoint() const
{
  return _targetHeaterTemp;
}

float iDryer::GetOutput() const
{
  return _output;
}

void iDryer::SetOutput(float output)
{
  _output = output;
  _overrideOutput = true;
}

bool iDryer::IsHeatingAllowed() const
{
  return state == DRY || state == STORAGE || state == AUTOPID;
}

void iDryer::Reset()
{
  _lastScreenUpdateTimestamp = 0;
  _targetHeaterTemp = 0;
  _output = 0;
  _overrideOutput = false;

  airPid.Reset();
  heaterPid.Reset();
}

bool iDryer::UpdateData()
{
  data.timestamp = millis();
  data.ntcTemp = (_heaterTempSensor.analog2temp() + data.ntcTemp) / 2.0f;

#ifdef SENSOR_SHT31
  if (_airTempSensor.dataReady())
  {
    _airTempSensor.read();
    data.airTemp = (_airTempSensor.getTemperature() + data.airTemp) / 2.0f;
    data.airHumidity = (_airTempSensor.getHumidity() + data.airHumidity) / 2.0f;
  }
#endif

#ifdef SENSOR_BME280
  data.airTemp = (_airTempSensor.readTemperature() + data.airTemp) / 2.0f;
  data.airHumidity = (_airTempSensor.readHumidity() + data.airHumidity) / 2.0f;
#endif

  data.airTempCorrected = data.airTemp;
  data.flagScreenUpdate = false;

  if (data.airTemp > MIN_CALIB_TEMP)
  {
    data.airTempCorrected = math::map_to_range_with_clamp(data.airTemp, MIN_CALIB_TEMP, MAX_CALIB_TEMP, REAL_CALIB_TEMP_MIN, REAL_CALIB_TEMP_MAX);
  }

  if (data.timestamp - _lastScreenUpdateTimestamp > SCREEN_UPADATE_TIME)
  {
    _lastScreenUpdateTimestamp = data.timestamp;
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

void iDryer::Setpoint()
{
  auto time = data.timestamp / float(math::msCountInSec); // Текущее время
  auto currentAirTemp = data.airTempCorrected;            // Текущая температура
  auto targetAirTemp = data.setTemp;                      // Заданная температура
  auto currentHeaterTemp = data.ntcTemp;                  // Текущая температура нагревателя

  auto airTempError = targetAirTemp - currentAirTemp;
  airPid.Process(time, airTempError);

  if (airPid.IsOutputUpdated())
  {
    _targetHeaterTemp = targetAirTemp;                              // Целевая температура нагревателя
    _targetHeaterTemp += airPid.GetMappedOutput(0.0f, data.deltaT); // Компенсация статической ошибки
  }

  if (_targetHeaterTemp > TMP_MAX)
  {
    _targetHeaterTemp = TMP_MAX;
  }

  // Отключение при критическом перегреве
  if (currentAirTemp >= targetAirTemp + CRITICAL_OVERHEAT)
  {
    _targetHeaterTemp = 0;
  }

  auto heaterTempError = _targetHeaterTemp - currentHeaterTemp;
  heaterPid.Process(time, heaterTempError);

  if (heaterPid.IsOutputUpdated())
  {
    if (!_overrideOutput)
    {
      _output = heaterPid.GetOutput();
    }

    if (_targetHeaterTemp == 0)
    {
      _output = 0;
    }
  }
}
