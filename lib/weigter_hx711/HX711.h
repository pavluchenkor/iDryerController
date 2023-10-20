#ifndef HX711_h
#define HX711_h
#include "Arduino.h"
#define MAX_SENSORS 4
#define RATE 1

#if RATE == 1
  #define RATE_DELAY 60
#elif RATE == 0
  #define RATE_DELAY 600
#endif


extern uint32_t zero_weight_eep[] EEMEM;
extern uint32_t offset_eep[] EEMEM;

class HX711
{
private:
  byte CLOCK_PIN;
  byte OUT_PIN;
  byte GAIN;
  bool pinsConfigured;
  float ratio;

public:
  int32_t zero_weight;
  int32_t offset;
  uint16_t tare;
  uint16_t mass;
  uint16_t mass_filter[5];

  HX711(){};
  HX711(byte output_pin, byte clock_pin);
  virtual ~HX711();
  bool readyToSend();
  void setGain(byte gain = 128);
  int32_t read();
  int16_t read_mass();
  void zeroSet(uint8_t avg_size);
  int32_t zeroGet();
  void tareSet(uint8_t avg_size);
  float tareGet();
  void offsetFirstSet(uint8_t avg_size);
  void offsetSet(int32_t offset_settings);
  int32_t offsetGet();
  uint16_t median(uint16_t newValue);
  void setupGain();
};

class HX711Multi
{
public:
  // HX711 _sensors[MAX_SENSORS];

  // HX711Multi(uint8_t numSensors, uint8_t *dtPins, uint8_t sckPins);
  HX711Multi(uint8_t numSensors, uint8_t dtPin, uint8_t sckPin, uint8_t aPin, uint8_t bPin);
  void begin(byte gain);
  // int32_t readSensor(int sensorIndex);
  int32_t getZeroWeight(uint8_t numSensor);
  // int32_t getOffset(uint8_t numSensor);
  // void setZeroWeight(int32_t value, uint8_t numSensor);
  // void setOffset(int32_t value, uint8_t numSensor);
  void zeroSetupMulti(uint8_t sensorNum);
  void offsetSetupMulti(uint8_t sensorNum);
  int32_t readMulti(uint8_t sensorIndex);
  // int16_t readMassMulti(uint8_t sensorNum);
  int16_t readMassMulti();
  void HX711Multi::updateFilter(uint8_t sensorNum, uint16_t newValue);
  void setGain(byte gain);
  void setupGainMulti(uint8_t sensorIndex);
  bool readyToSend(uint8_t sensorIndex);
  bool multiplexerPinSet(uint8_t sensorNum);
  int16_t getMassMulti(uint8_t sensorNum);
  void offsetFirstSetMulti(uint8_t avg_size, uint8_t sensorNum);
  void zeroSetMulti(uint8_t avg_size, uint8_t sensorNum);
  uint16_t tare[MAX_SENSORS];
  uint8_t sensorNum;

private:
  uint8_t _numSensors;
  uint8_t _prevNum;
  uint8_t _multiplexerPinSetFlag;
  // uint8_t _currentSensor;
  unsigned long _lastTime;
  // uint8_t *_dtPins;
  uint8_t _dtPin;
  uint8_t _sckPin;
  uint8_t _aPin;
  uint8_t _bPin;
  uint8_t GAIN;
  bool pinsConfigured;

  // int16_t mass[MAX_SENSORS];
  uint16_t mass_filter[MAX_SENSORS][5];
};

#endif /* HX711_h */