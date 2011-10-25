#ifndef GAZEBO_MATH_FUNCTIONS_HH
#define GAZEBO_MATH_FUNCTIONS_HH

#include <cmath>
#define EPSILON 0.0001

namespace gazebo
{
  namespace math
  {
    inline bool equal(const double &_a, const double &_b, 
                      const double &_epsilon)
    {
      return std::fabs(_a - _b) <= _epsilon;
    }

    inline bool equal(const float &_a, const float &_b, const float &_epsilon)
    {
      return std::fabs(_a - _b) <= _epsilon;
    }

    inline double precision(const double &_a, const unsigned int &_precision)
    {
      return round(_a * pow(10,_precision)) / pow(10,_precision);
    }

    inline float precision(const float &_a, const unsigned int &_precision)
    {
      return roundf(_a * pow(10,_precision)) / pow(10,_precision);
    }
  }
}
#endif
