#ifndef PID_H
#define PID_H

namespace math::algorithms
{
  class PIDController
  {
  private:
    // settings
    float _minDeltaTime = 0;
    float _proportionalGain = 0;
    float _integralGain = 0;
    float _derivativeGain = 0;
    float _filterGain = 0;

    // state
    float _previousTime = 0;
    float _previousValue = 0;

    // calculations
    float _deltaTime = 0;
    float _proportionalTerm = 0;
    float _integralTerm = 0;
    float _derivativeTerm = 0;
    float _filterTerm = 0;
    float _output = 0;

  public:
    PIDController();

    // getters
    float GetDeltaTime() const;
    float GetProportionalTerm() const;
    float GetIntegralTerm() const;
    float GetDerivativeTerm() const;
    float GetFilterTerm() const;
    float GetOutput() const;

    // setters
    void SetMinDeltaTime(float value);
    void SetProportionalGain(float value);
    void SetIntegralGain(float value);
    void SetDerivativeGain(float value);
    void SetFilterGain(float value);

    // calculations
    void Process(float time, float value);
  };
}

#endif // PID_H
