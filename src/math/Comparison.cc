#include <cmath>
#include <cstdlib>
#include <limits>
#include <assert.h>

#include "math/Comparison.hh"

#define EPSILON 0.0001

namespace gazebo
{
  namespace math
  {
    bool equal(const double &_a, const double &_b, const double &_epsilon)
    {
      return std::fabs(_a - _b) <= _epsilon;
    }

    bool equal(const float &_a, const float &_b, const float &_epsilon)
    {
      return std::fabs(_a - _b) <= _epsilon;
    }
  }
}
