#include "pid.h"

namespace math::algorithms
{
  PIDController::PIDController()
  {
  }

  bool PIDController::IsOutputUpdated() const
  {
    return _outputUpdated;
  }

  float PIDController::GetDeltaTime() const
  {
    return _deltaTime;
  }

  float PIDController::GetMinDeltaTime() const
  {
    return _minDeltaTime;
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

#if KASYAK_FINDER
  float PIDController::GetTime() const
  {
    return _time;
  }

  float PIDController::GetInput() const
  {
    return _input;
  }
#endif

  float PIDController::GetOutput() const
  {
    return math::clamp(_output, _minOutput, _maxOutput);
  }

  float PIDController::GetMappedOutput(float lowerBound, float upperBound) const
  {
    return math::map_to_range_with_clamp(_output, _minOutput, _maxOutput, lowerBound, upperBound);
  }

  float PIDController::GetMinOutput() const
  {
    return _minOutput;
  }

  float PIDController::GetMaxOutput() const
  {
    return _maxOutput;
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
    _outputUpdated = false;
#if KASYAK_FINDER
    _time = time;
    _input = value;
#endif
    _deltaTime = time - _previousTime;

    if (_deltaTime < 0.0f)
    {
      Reset();

      _previousTime = time;
      _previousInput = value;

      return;
    }

    if (_deltaTime < _minDeltaTime)
    {
      return;
    }

    _proportionalTerm = value * _proportionalGain;

    auto outputIsSaturating = _output != GetOutput();

    if (!outputIsSaturating)
    {
      _integralTerm += value * _integralGain * _deltaTime;
    }

    _integralTerm = math::clamp(_integralTerm, -_maxOutput, _maxOutput);

    auto a = 1.0f + 2.0f * _filterGain / _deltaTime;
    auto aPrev = 1.0f - 2.0f * _filterGain / _deltaTime;
    auto b = 2.0f / _deltaTime;
    auto bPrev = -2.0f / _deltaTime;

    _filterTerm = (value * b + _previousInput * bPrev - _filterTerm * aPrev) / a;
    _derivativeTerm = _filterTerm * _derivativeGain;

    _previousTime = time;
    _previousInput = value;

    _output = _proportionalTerm + _integralTerm + _derivativeTerm;
    _outputUpdated = true;
  }

  void PIDController::Reset()
  {
    _outputUpdated = false;
    _previousTime = 0;
    _previousInput = 0;

    _deltaTime = 0;
    _proportionalTerm = 0;
    _integralTerm = 0;
    _derivativeTerm = 0;
    _filterTerm = 0;
    _output = 0;
  }
}