#ifndef MATHEXTENSIONS_H
#define MATHEXTENSIONS_H
#include <stdlib.h>

namespace math
{
  constexpr auto msCountInSec = 1000.0f;

  float map_to_range(float value, float inputLowerBound, float inputUpperBound, float outputLowerBound, float outputUpperBound);

  float clamp(float value, float lowerBound, float upperBound);
}

#endif // MATHEXTENSIONS_H
