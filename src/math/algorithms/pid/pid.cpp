#include "pid.h"

namespace math::algorithms
{
  PIDController::PIDController()
  {
  }

  float PIDController::GetDeltaTime() const
  {
    return _deltaTime;
  }

  float PIDController::GetProportionalTerm() const
  {
    return _proportionalTerm;
  }

  float PIDController::GetIntegralTerm() const
  {
    return _integralTerm;
  }

  float PIDController::GetDerivativeTerm() const
  {
    return _derivativeTerm;
  }

  float PIDController::GetFilterTerm() const
  {
    return _filterTerm;
  }

  float PIDController::GetOutput() const
  {
    return _output;
  }

  void PIDController::SetMinDeltaTime(float value)
  {
    _minDeltaTime = value;
  }

  void PIDController::SetProportionalGain(float value)
  {
    _proportionalGain = value;
  }

  void PIDController::SetIntegralGain(float value)
  {
    _integralGain = value;
  }

  void PIDController::SetDerivativeGain(float value)
  {
    _derivativeGain = value;
  }

  void PIDController::SetFilterGain(float value)
  {
    _filterGain = value;
  }

  void PIDController::Process(float time, float value)
  {
    _deltaTime = time - _previousTime;

    if (_deltaTime < _minDeltaTime)
    {
      return;
    }

    _proportionalTerm = value * _proportionalGain;
    _integralTerm += value * _integralGain * _deltaTime;

    auto a = 1.0f + 2.0f * _filterGain / _deltaTime;
    auto aPrev = 1.0f - 2.0f * _filterGain / _deltaTime;
    auto b = 2.0f / _deltaTime;
    auto bPrev = -2.0f / _deltaTime;

    _filterTerm = (value * b + _previousValue * bPrev - _filterTerm * aPrev) / a;
    _derivativeTerm = _filterTerm * _derivativeGain;

    _previousTime = time;
    _previousValue = value;

    _output = _proportionalTerm + _integralTerm + _derivativeTerm;
  }
}