#ifndef PID_H
#define PID_H
#include "Configuration.h"
#include "math/math_extensions.h"

namespace math::algorithms
{
  class PIDController
  {
  private:
    // settings
    const float _minOutput = 0;
    const float _maxOutput = 1;
    float _minDeltaTime = 0;
    float _proportionalGain = 0;
    float _integralGain = 0;
    float _derivativeGain = 0;
    float _filterGain = 0;

    // state
    bool _outputUpdated = false;
#if KASYAK_FINDER
    float _time = 0;
    float _input = 0;
#endif
    float _previousTime = 0;
    float _previousInput = 0;

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
    bool IsOutputUpdated() const;
    float GetDeltaTime() const;
    float GetMinDeltaTime() const;
    float GetProportionalTerm() const;
    float GetIntegralTerm() const;
    float GetDerivativeTerm() const;
    float GetFilterTerm() const;
#if KASYAK_FINDER
    float GetTime() const;
    float GetInput() const;
#endif
    float GetOutput() const;
    float GetMappedOutput(float lowerBound, float upperBound) const;
    float GetMinOutput() const;
    float GetMaxOutput() const;

    // setters
    void SetMinDeltaTime(float value);
    void SetProportionalGain(float value);
    void SetIntegralGain(float value);
    void SetDerivativeGain(float value);
    void SetFilterGain(float value);

    // calculations
    void Process(float time, float value);

    void Reset();
  };
}

#endif // PID_H
