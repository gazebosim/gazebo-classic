#ifndef GAZEBO_MATH_COMPARISON_HH
#define GAZEBO_MATH_COMPARISON_HH

namespace gazebo
{
  namespace math
  {
    bool equal(const double &_a, const double &_b, const double &_epsilon);
    bool equal(const float &_a, const float &_b, const float &_epsilon);
  }
}
#endif
