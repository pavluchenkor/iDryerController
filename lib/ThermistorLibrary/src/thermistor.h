
#ifndef THERMISTOR_H
  #define THERMISTOR_H


class thermistor
{
  public:
    thermistor(int pin, int sensorNumber);
    float analog2temp();
    float analog2tempEADC(int adcValue);
  private:
    int _pin;
    int _sensorNumber;
};



#endif
