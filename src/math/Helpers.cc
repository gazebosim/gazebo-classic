#include <cmath>
#include <math.h>
#include <cstdlib>
#include <limits>
#include <assert.h>

#include "math/Helpers.hh"

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

    double precision(const double &_a, const unsigned int &_precision)
    {
      return round(_a * pow(10,_precision)) / pow(10,_precision);
    }

    float precision(const float &_a, const unsigned int &_precision)
    {
      return roundf(_a * pow(10,_precision)) / pow(10,_precision);
    }

  }
}
