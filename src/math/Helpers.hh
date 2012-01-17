#ifndef GAZEBO_MATH_FUNCTIONS_HH
#define GAZEBO_MATH_FUNCTIONS_HH

#include <cmath>
#include <limits>
#include <string>
#include <iostream>

namespace gazebo
{
  namespace math
  {
    static const double NAN_D = std::numeric_limits<double>::quiet_NaN();
    static const double NAN_I = std::numeric_limits<int>::quiet_NaN();

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
      return round(_a * pow(10, _precision)) / pow(10, _precision);
    }

    inline float precision(const float &_a, const unsigned int &_precision)
    {
      return roundf(_a * pow(10, _precision)) / pow(10, _precision);
    }

    inline int parseInt(const std::string& input)
    {
      const char *p = input.c_str();
      if (!*p || *p == '?')
        return NAN_I;

      int s = 1;
      while (*p == ' ')
        p++;

      if (*p == '-')
      {
        s = -1;
        p++;
      }

      double acc = 0;
      while (*p >= '0' && *p <= '9')
        acc = acc * 10 + *p++ - '0';

      if (*p)
      {
        std::cerr << "Invalid int numeric format[" << input << "]\n";
        return 0.0;
      }

      return s * acc;
    }

    inline double parseFloat(const std::string& input)
    {
      const char *p = input.c_str();
      if (!*p || *p == '?')
        return NAN_D;
      int s = 1;
      while (*p == ' ')
        p++;

      if (*p == '-')
      {
        s = -1;
        p++;
      }

      double acc = 0;
      while (*p >= '0' && *p <= '9')
        acc = acc * 10 + *p++ - '0';

      if (*p == '.')
      {
        double k = 0.1;
        p++;
        while (*p >= '0' && *p <= '9')
        {
          acc += (*p++ - '0') * k;
          k *= 0.1;
        }
      }
      if (*p == 'e')
      {
        int es = 1;
        int f = 0;
        p++;
        if (*p == '-')
        {
          es = -1;
          p++;
        }
        else if (*p == '+')
        {
          es = 1;
          p++;
        }
        while (*p >= '0' && *p <= '9')
          f = f * 10 + *p++ - '0';

        acc *= pow(10, f*es);
      }

      if (*p)
      {
        std::cerr << "Invalid double numeric format[" << input << "]\n";
        return 0.0;
      }
      return s * acc;
    }
  }
}
#endif


