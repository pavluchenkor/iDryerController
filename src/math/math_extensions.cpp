#include "math_extensions.h"

namespace math
{
  float map_to_range_with_clamp(float value, float inputLowerBound, float inputUpperBound, float outputLowerBound, float outputUpperBound)
  {
    value = math::clamp(value, inputLowerBound, inputUpperBound);

    return map_to_range(value, inputLowerBound, inputUpperBound, outputLowerBound, outputUpperBound);
  }

  float map_to_range(float value, float inputLowerBound, float inputUpperBound, float outputLowerBound, float outputUpperBound)
  {
    auto slope = (outputUpperBound - outputLowerBound) / (inputUpperBound - inputLowerBound);

    return outputLowerBound + slope * (value - inputLowerBound);
  }

  float clamp(float value, float lowerBound, float upperBound)
  {
    return (value < lowerBound) ? lowerBound : (upperBound < value) ? upperBound : value;
  }
}