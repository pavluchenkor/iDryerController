#include <Arduino.h>
#include "HX711.h"
#include <EEPROM.h>

float calibration_mass = 1000;

uint32_t zero_weight_eep[] EEMEM
{
    0,
    0,
    0,
    0,
};
uint32_t offset_eep[] EEMEM
{
    0,
    0,
    0,
    0,
};


HX711Multi::HX711Multi(uint8_t numSensors, uint8_t *dtPins, uint8_t sckPins)
{
  //Serial.begin(57600);
  _numSensors = numSensors;
  _dtPins = dtPins;
  _sckPin = sckPins;
  for (int i = 0; i < _numSensors; i++)
  {
    _sensors[i] = HX711(_dtPins[i], _sckPin);

    _sensors[i].zero_weight = getZeroWeight(i);
    _sensors[i].offset = getOffset(i);
  }
}

int32_t HX711Multi::getZeroWeight(uint8_t numSensor)
{
  // Чтение значения из EEPROM
  return static_cast<int32_t>(eeprom_read_dword(&zero_weight_eep[numSensor]));
}

void HX711Multi::setZeroWeight(int32_t value, uint8_t numSensor)
{
  // //Serial.println("Запись значения в EEPROM setZeroWeight");
  eeprom_write_dword(&zero_weight_eep[numSensor], static_cast<uint32_t>(value));
  // //Serial.println("Запись значения в EEPROM setZeroWeight завершена");
}

int32_t HX711Multi::getOffset(uint8_t numSensor)
{
  // Чтение значения из EEPROM
  return static_cast<int32_t>(eeprom_read_dword(&offset_eep[numSensor]));
}

void HX711Multi::setOffset(int32_t value, uint8_t numSensor)
{
  // Запись значения в EEPROM
  eeprom_write_dword(&offset_eep[numSensor], static_cast<uint32_t>(value));
}

int32_t HX711Multi::readSensor(int sensorIndex)
{
  if (sensorIndex >= 0 && sensorIndex < _numSensors)
  {
    return _sensors[sensorIndex].read();
  }
  return 0;
}

void HX711Multi::zero_setup(uint8_t sensorNum)
{
  _sensors[sensorNum].zeroSet(100);
  // setZeroWeight(sensorNum, _sensors[sensorNum].zero_weight);
}

void HX711Multi::offset_setup(uint8_t sensorNum)
{
  _sensors[sensorNum].offsetFirstSet(100);
  // setOffset(sensorNum, _sensors[sensorNum].offset);
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
  //Serial.println("Add Calibrated Mass 1");
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

int32_t HX711::read()
{
  while (!readyToSend())
    ;

  byte data[3];

  for (byte j = 3; j--;)
  {
    data[j] = shiftIn(OUT_PIN, CLOCK_PIN, MSBFIRST);
  }

  // set gain
  for (int i = 0; i < GAIN; i++)
  {
    digitalWrite(CLOCK_PIN, HIGH);
    digitalWrite(CLOCK_PIN, LOW);
  }

  data[2] ^= 0x80;
  return ((uint32_t)data[2] << 16) | ((uint32_t)data[1] << 8) | (uint32_t)data[0];
}

float HX711::median(float newValue)
{
  for (int i = 4; i > 0; i--)
  {
    mass_fliter[i] = mass_fliter[i - 1];
  }
  mass_fliter[0] = newValue;

  for (int i = 0; i < 5; i++)
  {
    for (int j = i + 1; j < 5; j++)
    {
      if (mass_fliter[i] > mass_fliter[j])
      {
        float temp = mass_fliter[i];
        mass_fliter[i] = mass_fliter[j];
        mass_fliter[j] = temp;
      }
    }
  }

  return mass_fliter[2];
}


float HX711::read_mass()
{
  // mass_fliter[1] = 10;
  // mass += calibration_mass * ((float)(read() - zero_weight) / (float)(offset - zero_weight));  
  // mass /= 2.0;
  // mass = abs(mass) < 1 ? 0 : (mass * 2 + 0.5) / 2.0 - tare;
  // return mass;

  float newValue = calibration_mass * ((float)(read() - zero_weight) / (float)(offset - zero_weight));
  float medianValue = median(newValue);
  medianValue = abs(medianValue) < 1 ? 0 : (medianValue * 2 + 0.5) / 2.0 - tare;
  return medianValue;
}
