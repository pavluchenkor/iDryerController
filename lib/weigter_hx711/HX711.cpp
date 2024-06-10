#include <Arduino.h>
#include "HX711.h"
// #include <EEPROM.h>
#define CALIBRATION_MASS 1000

uint32_t zero_weight_eep[] EEMEM{
    0,
    0,
    0,
    0,
};
uint32_t offset_eep[] EEMEM{
    0,
    0,
    0,
    0,
};

float calibration_mass = CALIBRATION_MASS;

HX711Multi::HX711Multi(uint8_t numSensors, uint8_t dtPin, uint8_t sckPin, uint8_t aPin, uint8_t bPin)
{
  // Serial.begin(57600);
  _numSensors = numSensors;
  _dtPin = dtPin;
  _sckPin = sckPin;
  _aPin = aPin;
  _bPin = bPin;
  _multiplexerPinSetFlag = false;
  sensorNum = 0;
  _prevNum = _numSensors + 1;

  pinMode(sckPin, OUTPUT);
  pinMode(dtPin, INPUT);
  pinMode(aPin, OUTPUT);
  pinMode(bPin, OUTPUT);

  // setGain(128);

  // for (int i = 0; i < _numSensors; i++)
  // {
  //   // _sensors[i] = HX711(_dtPins[i], _sckPin);
  //   zero_weight[i] = getZeroWeight(i);
  //   offset[i] = getOffset(i);
  // }
}



void HX711Multi::begin(byte gain)
{
  digitalWrite(_sckPin, LOW);
  digitalWrite(_dtPin, LOW);
  digitalWrite(_aPin, LOW);
  digitalWrite(_bPin, LOW);
  setGain(gain);
}

void HX711Multi::setGain(byte gain)
{
  switch (gain)
  {
  case 128:
    GAIN = 1;
    break;
  case 64:
    GAIN = 3;
    break;
  case 32:
    GAIN = 2;
    break;
  }

  digitalWrite(_sckPin, LOW);
  readMulti(0);
}

void HX711Multi::setupGainMulti(uint8_t sensorNum)
{
  // for (int i = 0; i < GAIN; i++)
  for (int i = 0; i < 1; i++)
  {
    digitalWrite(_sckPin, HIGH);
    digitalWrite(_sckPin, LOW);
  }
}


int16_t HX711Multi::readMassMulti()
{
  if (readyToSend(sensorNum))
  {
    for (uint8_t i = 4; i > 0; i--)
    {
      mass_filter[sensorNum][i] = mass_filter[sensorNum][i - 1];
    }

    for (uint8_t i = 0; i < 5; i++)
    {
      for (uint8_t j = i + 1; j < 5; j++)
      {
        if (mass_filter[sensorNum][i] > mass_filter[sensorNum][j])
        {
          int16_t temp = mass_filter[sensorNum][i];
          mass_filter[sensorNum][i] = mass_filter[sensorNum][j];
          mass_filter[sensorNum][j] = temp;
        }
      }
    }

    mass_filter[sensorNum][0] =
        uint16_t((float)calibration_mass *
                 (float)(readMulti(sensorNum) - eeprom_read_dword(&zero_weight_eep[sensorNum])) /
                 (float)(eeprom_read_dword(&offset_eep[sensorNum]) - eeprom_read_dword(&zero_weight_eep[sensorNum])));
    if (mass_filter[sensorNum][0] > 2500)
      mass_filter[sensorNum][0] = 0;

    sensorNum++;
    if (sensorNum >= _numSensors)
      sensorNum = 0;

    return mass_filter[sensorNum][0];
  }
  return 0;
}

int16_t HX711Multi::getMassMulti(uint8_t sensorNum)
{
  return mass_filter[sensorNum][2] - tare[sensorNum];
}

int32_t HX711Multi::readMulti(uint8_t sensorNum)
{
  while (!readyToSend(sensorNum))
    ;

  byte data[3];

  for (byte j = 3; j--;)
  {
    data[j] = shiftIn(_dtPin, _sckPin, MSBFIRST);
  }
  setupGainMulti(sensorNum);

  data[2] ^= 0x80;
  return ((uint32_t)data[2] << 16) | ((uint32_t)data[1] << 8) | (uint32_t)data[0];
  // }
  // else
  // {
  //   return 0;
  // }
}

bool HX711Multi::readyToSend(uint8_t sensorNum)
{
  if (multiplexerPinSet(sensorNum))
  {
    return digitalRead(_dtPin) == LOW;
  }
  else
  {
    return false;
  }
}

bool HX711Multi::multiplexerPinSet(uint8_t sensorNum)
{
  if (_prevNum != sensorNum)
  {
    digitalWrite(_aPin, (sensorNum >> 0) & 1);
    digitalWrite(_bPin, (sensorNum >> 1) & 1);
    _prevNum = sensorNum;
    _lastTime = millis();
    return false;
  }
  if (_prevNum == sensorNum && millis() - _lastTime > RATE_DELAY)
  {
    return true;
  }
  return false;
}

void HX711Multi::zeroSetupMulti(uint8_t sensorNum)
{
  zeroSetMulti(100, sensorNum);
}

void HX711Multi::offsetSetupMulti(uint8_t sensorNum)
{
  offsetFirstSetMulti(100, sensorNum);
}

void HX711Multi::offsetFirstSetMulti(uint8_t avg_size, uint8_t sensorNum)
{
  while (!readyToSend(sensorNum))
    ;

  int32_t offset = 0;

  for (int i = 0; i < int(avg_size); i++)
  {
    delay(20);
    offset += readMulti(sensorNum);
  }
  offset /= int32_t(avg_size);
  eeprom_write_dword(&offset_eep[sensorNum], static_cast<uint32_t>(offset));
}

void HX711Multi::zeroSetMulti(uint8_t avg_size, uint8_t sensorNum)
{
  while (!readyToSend(sensorNum))
    ;
  int32_t zero_weight = 0;

  for (int i = 0; i < int(avg_size); i++)
  {
    delay(20);
    zero_weight += readMulti(sensorNum);
  }
  zero_weight /= int32_t(avg_size);
  eeprom_write_dword(&zero_weight_eep[sensorNum], static_cast<uint32_t>(zero_weight));
}

HX711::HX711(byte output_pin, byte clock_pin)
{
  CLOCK_PIN = clock_pin;
  OUT_PIN = output_pin;
  GAIN = 1;
  pinsConfigured = false;
  zero_weight = 0L;
  offset = 0L;
  tare = 0;
  // ratio = 0.0;
  mass = 0;
}

HX711::~HX711()
{
}

bool HX711::readyToSend()
{
  if (!pinsConfigured)
  {
    // We need to set the pin mode once, but not in the constructor
    pinMode(CLOCK_PIN, OUTPUT);
    pinMode(OUT_PIN, INPUT);
    pinsConfigured = true;
  }
  return digitalRead(OUT_PIN) == LOW;
}

void HX711::setGain(byte gain)
{
  switch (gain)
  {
  case 128:
    GAIN = 1;
    break;
  case 64:
    GAIN = 3;
    break;
  case 32:
    GAIN = 2;
    break;
  }

  digitalWrite(CLOCK_PIN, LOW);
  read();
}

void HX711::zeroSet(uint8_t avg_size)
{
  zero_weight = 0;
  for (int i = 0; i < int(avg_size); i++)
  {
    delay(10);
    zero_weight += read();
  }
  zero_weight /= int32_t(avg_size);
}

int32_t HX711::zeroGet()
{
  return zero_weight;
}

void HX711::tareSet(uint8_t avg_size)
{
  for (int i = 0; i < int(avg_size); i++)
  {
    delay(10);
    tare += read_mass();
  }
  zero_weight /= int32_t(avg_size);
}

float HX711::tareGet()
{
  return tare;
}

void HX711::offsetFirstSet(uint8_t avg_size)
{
  // //Serial.println("Remove  Calibrated Mass");
  delay(2000);
  // Serial.println("Add Calibrated Mass 1");
  while (true)
  {
    // //Serial.print(read());
    // //Serial.print("\tzero_weight + 10000: ");
    // //Serial.println(zero_weight + 10000);

    // if (read() < zero_weight + 10000)
    // {
    // }
    // else
    // {
    // delay(2000);
    for (int i = 0; i < int(avg_size); i++)
    {
      offset += read();
    }
    offset /= int32_t(avg_size);
    break;
    // }
  }
  // ratio = ((float)(reading - zero_weight) / (float)(offset - zero_weight));
  // //Serial.println("Calibration Complete");
}

void HX711::offsetSet(int32_t offset_settings)
{
  offset = offset_settings;
}

int32_t HX711::offsetGet()
{
  return offset;
}

void HX711::setupGain()
{
  for (int i = 0; i < GAIN; i++)
  {
    digitalWrite(CLOCK_PIN, HIGH);
    digitalWrite(CLOCK_PIN, LOW);
  }
}

int32_t HX711::read()
{
  while (!readyToSend())
    ;

  byte data[3];

  for (byte j = 3; j--;)
  {
    data[j] = shiftIn(OUT_PIN, CLOCK_PIN, MSBFIRST);
  }

  setupGain();

  data[2] ^= 0x80;
  return ((uint32_t)data[2] << 16) | ((uint32_t)data[1] << 8) | (uint32_t)data[0];
}

// uint16_t HX711::median(uint16_t newValue)
// {
//   for (uint8_t i = 4; i > 0; i--)
//   {
//     mass_filter[i] = mass_filter[i - 1];
//   }
//   mass_filter[0] = newValue;
//   for (uint8_t i = 0; i < 5; i++)
//   {
//     for (uint8_t j = i + 1; j < 5; j++)
//     {
//       if (mass_filter[i] > mass_filter[j])
//       {
//         uint16_t temp = mass_filter[i];
//         mass_filter[i] = mass_filter[j];
//         mass_filter[j] = temp;
//       }
//     }
//   }
//   return mass_filter[2];
// }

// // uint16_t HX711::median(uint16_t newValue)
// // {
// //   for (int i = 4; i > 0; i--)
// //   {
// //     mass_filter[i] = mass_filter[i - 1];
// //   }
// //   mass_filter[0] = newValue;
// //   // Create an array of indices
// //   uint16_t indices[] = {0, 1, 2, 3, 4};
// //   // Sort the indices based on the values in mass_filter
// //   for (uint8_t i = 0; i < 5; i++)
// //   {
// //     for (uint8_t j = i + 1; j < 5; j++)
// //     {
// //       if (mass_filter[indices[i]] > mass_filter[indices[j]])
// //       {
// //         uint8_t temp = indices[i];
// //         indices[i] = indices[j];
// //         indices[j] = temp;
// //       }
// //     }
// //   }
// //   return mass_filter[indices[2]];
// // }

// uint16_t HX711::read_mass()
// {
//   // mass += calibration_mass * ((float)(read() - zero_weight) / (float)(offset - zero_weight));
//   // mass /= 2.0;
//   // mass = abs(mass) < 1 ? 0 : (mass * 2 + 0.5) / 2.0 - tare;
//   // return mass;
//   mass = uint16_t(calibration_mass * ((float)(read() - zero_weight) / (float)(offset - zero_weight)));
//   uint16_t medianValue = median(mass);
//   mass = abs(medianValue) < 1 ? 0 : (medianValue * 2 + 0.5) / 2.0 - tare;
//   return mass;
// }

int16_t HX711::read_mass()
{
  // Чтение значения
  mass += int16_t(calibration_mass * ((float)(read() - zero_weight) / (float)(offset - zero_weight)));
  mass /= 2;

  // Обновление медианного фильтра
  for (uint8_t i = 4; i > 0; i--)
  {
    mass_filter[i] = mass_filter[i - 1];
  }
  mass_filter[0] = mass;

  for (uint8_t i = 0; i < 5; i++)
  {
    for (uint8_t j = i + 1; j < 5; j++)
    {
      if (mass_filter[i] > mass_filter[j])
      {
        int16_t temp = mass_filter[i];
        mass_filter[i] = mass_filter[j];
        mass_filter[j] = temp;
      }
    }
  }

  // uint16_t medianValue = mass_filter[2];
  // mass = abs(medianValue) < 1 ? 0 : (medianValue * 2 + 0.5) / 2.0 - tare;

  return mass_filter[2];
}