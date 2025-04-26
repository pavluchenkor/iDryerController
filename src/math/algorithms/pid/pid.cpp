#include "pid.h"

namespace math::algorithms
{
  PIDController::PIDController()
  {
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
    _filterTerm += _filterTerm * _filterGain * _deltaTime;
    _derivativeTerm = value * _derivativeGain - _filterTerm;

    _previousTime = time;
    _previousValue = value;

    _output = _proportionalTerm + _integralTerm + _derivativeTerm;
  }
}