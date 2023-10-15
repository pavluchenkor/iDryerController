#ifndef HX711_h
#define HX711_h
#include "Arduino.h"
#define MAX_SENSORS 4

extern uint32_t zero_weight_eep[] EEMEM;
extern uint32_t offset_eep[] EEMEM;


class HX711
{
private:
  byte CLOCK_PIN;
  byte OUT_PIN;
  byte GAIN;
  bool pinsConfigured;
  // float ratio;

public:
  int32_t zero_weight;
  int32_t offset;
  uint16_t tare;
  float mass;
  // float mass_fliter[5];

  HX711(){};
  HX711(byte output_pin, byte clock_pin);
  virtual ~HX711();
  bool readyToSend();
  void setGain(byte gain = 128);
  int32_t read();
  float read_mass();
  void zeroSet(uint8_t avg_size);
  int32_t zeroGet();
  void tareSet(uint8_t avg_size);
  float tareGet();
  void offsetFirstSet(uint8_t avg_size);
  void offsetSet(int32_t offset_settings);
  int32_t offsetGet();
  float median(float newValue);
};

class HX711Multi
{
public:
  HX711 _sensors[MAX_SENSORS];

  HX711Multi(uint8_t numSensors, uint8_t *dtPins, uint8_t sckPins);
  int32_t readSensor(int sensorIndex);
  int32_t getZeroWeight(uint8_t numSensor);
  void setZeroWeight(int32_t value, uint8_t numSensor);
  int32_t getOffset(uint8_t numSensor);
  void setOffset(int32_t value, uint8_t numSensor);
  void zero_setup(uint8_t sensorNum);
  void offset_setup(uint8_t sensorNum);
private:
  uint8_t _numSensors;
  uint8_t *_dtPins;
  uint8_t _sckPin;
};

#endif /* HX711_h */