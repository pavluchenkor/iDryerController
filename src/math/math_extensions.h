#ifndef MATHEXTENSIONS_H
#define MATHEXTENSIONS_H
#include <stdlib.h>
#include <stdint.h>

namespace math
{
  constexpr float pi = 3.1415927f;
  constexpr float rd = 180.0f / pi;
  constexpr uint32_t msCountInSec = 1000;
  constexpr uint32_t usCountInSec = 1000000;
  constexpr uint32_t nsCountInSec = 1000000000;

  float map_to_range_with_clamp(float value, float inputLowerBound, float inputUpperBound, float outputLowerBound, float outputUpperBound);

  float map_to_range(float value, float inputLowerBound, float inputUpperBound, float outputLowerBound, float outputUpperBound);

  float normalize(float value, float lowerBound, float upperBound);

  float clamp(float value, float lowerBound, float upperBound);

  float sign(float value);
}

#endif // MATHEXTENSIONS_H
