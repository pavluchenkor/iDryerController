#include "math_extensions.h"

namespace math
{
  float map_to_range(float value, float inputLowerBound, float inputUpperBound, float outputLowerBound, float outputUpperBound)
  {
    auto slope = 1.0f * (outputUpperBound - outputLowerBound) / (inputUpperBound - inputLowerBound);

    return outputLowerBound + slope * (math::clamp(value, inputLowerBound, inputUpperBound) - inputLowerBound);
  }

  float clamp(float value, float lowerBound, float upperBound)
  {
    return (value < lowerBound) ? lowerBound : (upperBound < value) ? upperBound : value;
  }
}