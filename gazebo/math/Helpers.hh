/*
 * Copyright 2011 Nate Koenig
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef GAZEBO_MATH_FUNCTIONS_HH
#define GAZEBO_MATH_FUNCTIONS_HH

#include <boost/math/special_functions/fpclassify.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <iostream>
#include <vector>

#define GZ_DBL_MAX std::numeric_limits<double>::max()
#define GZ_DBL_MIN std::numeric_limits<double>::min()

#define GZ_FLT_MAX std::numeric_limits<float>::max()
#define GZ_FLT_MIN std::numeric_limits<float>::min()

namespace gazebo
{
  namespace math
  {
    /// \addtogroup gazebo_math
    /// \brief A set of classes that encapsulate math related properties and
    ///        functions.
    /// \{

    static const double NAN_D = std::numeric_limits<double>::quiet_NaN();
    static const double NAN_I = std::numeric_limits<int>::quiet_NaN();

    /// \brief simple clamping function
    template<typename T>
    inline T clamp(T _v, T _min, T _max)
    {
      return std::max(std::min(_v, _max), _min);
    }

    /// \brief check if a float is NaN
    inline bool isnan(float _v)
    {
      return (boost::math::isnan)(_v);
    }

    /// \brief check if a double is NaN
    inline bool isnan(double _v)
    {
      return (boost::math::isnan)(_v);
    }

    /// \brief get mean of vector of values
    template<typename T>
    inline T mean(const std::vector<T> &_values)
    {
      T sum = 0;
      for (unsigned int i = 0; i < _values.size(); ++i)
        sum += _values[i];
      return sum / _values.size();
    }

    /// \brief get variance of vector of values
    template<typename T>
    inline T variance(const std::vector<T> &_values)
    {
      T avg = mean<T>(_values);

      T sum = 0;
      for (unsigned int i = 0; i < _values.size(); ++i)
        sum += (_values[i] - avg) * (_values[i] - avg);
      return sum / _values.size();
    }

    /// \brief get variance of vector of values
    template<typename T>
    inline T max(const std::vector<T> &_values)
    {
      T max = std::numeric_limits<T>::min();
      for (unsigned int i = 0; i < _values.size(); ++i)
        if (_values[i] > max)
          max = _values[i];
      return max;
    }

    /// \brief get min of vector of values
    template<typename T>
    inline T min(const std::vector<T> &_values)
    {
      T min = std::numeric_limits<T>::max();
      for (unsigned int i = 0; i < _values.size(); ++i)
        if (_values[i] < min)
          min = _values[i];
      return min;
    }

    /// \brief check if two values are equal in the machine precision sense
    template<typename T>
    inline bool equal(const T &_a, const T &_b,
                      const T &_epsilon = 1e-6)
    {
      return std::fabs(_a - _b) <= _epsilon;
    }

    /// \brief get precision of a value
    template<typename T>
    inline T precision(const T &_a, const unsigned int &_precision)
    {
      return round(_a * pow(10, _precision)) / pow(10, _precision);
    }

    /// \brief is this a power of 2?
    inline bool isPowerOfTwo(unsigned int _x)
    {
      return ((_x != 0) && ((_x & (~_x + 1)) == _x));
    }

    /// \brief parse string into int
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

    /// \brief parse string into float
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
    /// \}
  }
}
#endif
