/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_MATH_FUNCTIONS_HH_
#define _GAZEBO_MATH_FUNCTIONS_HH_

#include <boost/math/special_functions/fpclassify.hpp>
#include <boost/math/special_functions/round.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <iostream>
#include <vector>
#include "gazebo/util/system.hh"

/// \brief Double maximum value
#define GZ_DBL_MAX std::numeric_limits<double>::max()

/// \brief Double min value
#define GZ_DBL_MIN std::numeric_limits<double>::min()

/// \brief Double positive infinite value
#define GZ_DBL_INF std::numeric_limits<double>::infinity()

/// \brief Float maximum value
#define GZ_FLT_MAX std::numeric_limits<float>::max()

/// \brief Float minimum value
#define GZ_FLT_MIN std::numeric_limits<float>::min()

/// \brief 32bit unsigned integer maximum value
#define GZ_UINT32_MAX std::numeric_limits<uint32_t>::max()

/// \brief 32bit unsigned integer minimum value
#define GZ_UINT32_MIN std::numeric_limits<uint32_t>::min()

/// \brief 32bit integer maximum value
#define GZ_INT32_MAX std::numeric_limits<int32_t>::max()

/// \brief 32bit integer minimum value
#define GZ_INT32_MIN std::numeric_limits<int32_t>::min()


namespace gazebo
{
  namespace math
  {
    /// \addtogroup gazebo_math
    /// \brief A set of classes that encapsulate math related properties and
    ///        functions.
    /// \{

    /// \brief Returns the representation of a quiet not a number (NAN)
    static const double NAN_D = std::numeric_limits<double>::quiet_NaN();

    /// \brief Returns the representation of a quiet not a number (NAN)
    static const int NAN_I = std::numeric_limits<int>::quiet_NaN();

    /// \brief Simple clamping function
    /// \param[in] _v value
    /// \param[in] _min minimum
    /// \param[in] _max maximum
    template<typename T>
    inline T clamp(T _v, T _min, T _max)
    {
      return std::max(std::min(_v, _max), _min);
    }

    /// \brief check if a float is NaN
    /// \param[in] _v the value
    /// \return true if _v is not a number, false otherwise
    /// \deprecated See ignition::math::isnan
    inline bool
    GAZEBO_DEPRECATED(8.0)
    isnan(float _v)
    {
      return (boost::math::isnan)(_v);
    }

    /// \brief check if a double is NaN
    /// \param[in] _v the value
    /// \return true if _v is not a number, false otherwise
    /// \deprecated See ignition::math::isnan
    inline bool
    GAZEBO_DEPRECATED(8.0)
    isnan(double _v)
    {
      return (boost::math::isnan)(_v);
    }

#ifndef _WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
    /// \brief Fix a nan value.
    /// \param[in] _v Value to correct.
    /// \return 0 if _v is NaN, _v otherwise.
    /// \deprecated See ignition::math::fixnan
    inline float
    GAZEBO_DEPRECATED(8.0)
    fixnan(float _v)
    {
      return isnan(_v) || std::isinf(_v) ? 0.0f : _v;
    }

    /// \brief Fix a nan value.
    /// \param[in] _v Value to correct.
    /// \return 0 if _v is NaN, _v otherwise.
    /// \deprecated See ignition::math::fixnan
    inline double
    GAZEBO_DEPRECATED(8.0)
    fixnan(double _v)
    {
      return isnan(_v) || std::isinf(_v) ? 0.0 : _v;
    }
#ifndef _WIN32
#pragma GCC diagnostic pop
#endif

    /// \brief get mean of vector of values
    /// \param[in] _values the vector of values
    /// \return the mean
    /// \deprecated See ignition::math::mean
    template<typename T>
    inline T
    GAZEBO_DEPRECATED(8.0)
    mean(const std::vector<T> &_values)
    {
      T sum = 0;
      for (unsigned int i = 0; i < _values.size(); ++i)
        sum += _values[i];
      return sum / _values.size();
    }

    /// \brief get variance of vector of values
    /// \param[in] _values the vector of values
    /// \return the squared deviation
    /// \deprecated See ignition::math::variance
    template<typename T>
    inline T
    GAZEBO_DEPRECATED(8.0)
    variance(const std::vector<T> &_values)
    {
      T avg = mean<T>(_values);

      T sum = 0;
      for (unsigned int i = 0; i < _values.size(); ++i)
        sum += (_values[i] - avg) * (_values[i] - avg);
      return sum / _values.size();
    }

    /// \brief get the maximum value of vector of values
    /// \param[in] _values the vector of values
    /// \return maximum
    /// \deprecated See ignition::math::max
    template<typename T>
    inline T
    GAZEBO_DEPRECATED(8.0)
    max(const std::vector<T> &_values)
    {
      T max = std::numeric_limits<T>::min();
      for (unsigned int i = 0; i < _values.size(); ++i)
        if (_values[i] > max)
          max = _values[i];
      return max;
    }

    /// \brief get the minimum value of vector of values
    /// \param[in] _values the vector of values
    /// \return minimum
    /// \deprecated See ignition::math::min
    template<typename T>
    inline T
    GAZEBO_DEPRECATED(8.0)
    min(const std::vector<T> &_values)
    {
      T min = std::numeric_limits<T>::max();
      for (unsigned int i = 0; i < _values.size(); ++i)
        if (_values[i] < min)
          min = _values[i];
      return min;
    }

    /// \brief check if two values are equal, within a tolerance
    /// \param[in] _a the first value
    /// \param[in] _b the second value
    /// \param[in] _epsilon the tolerance
    template<typename T>
    inline bool equal(const T &_a, const T &_b,
                      const T &_epsilon = 1e-6)
    {
      return std::fabs(_a - _b) <= _epsilon;
    }

    /// \brief get value at a specified precision
    /// \param[in] _a the number
    /// \param[in] _precision the precision
    /// \return the value for the specified precision
    /// \deprecated See ignition::math::precision
    template<typename T>
    inline T
    GAZEBO_DEPRECATED(8.0)
    precision(const T &_a, const unsigned int &_precision)
    {
      if (!std::isinf(_a))
      {
        return boost::math::round(
          _a * pow(10, _precision)) / pow(10, _precision);
      }
      else
      {
        return _a;
      }
    }

    /// \brief is this a power of 2?
    /// \param[in] _x the number
    /// \return true if _x is a power of 2, false otherwise
    /// \deprecated See ignition::math::isPowerOfTwo
    inline bool
    GAZEBO_DEPRECATED(8.0)
    isPowerOfTwo(unsigned int _x)
    {
      return ((_x != 0) && ((_x & (~_x + 1)) == _x));
    }

    /// \brief Get the smallest power of two that is greater or equal to a given
    /// value
    /// \param[in] _x the number
    /// \return the same value if _x is already a power of two. Otherwise,
    /// it returns the smallest power of two that is greater than _x
    /// \deprecated See ignition::math::roundUpPowerOfTwo
    inline unsigned int
    GAZEBO_DEPRECATED(8.0)
    roundUpPowerOfTwo(unsigned int _x)
    {
      if (_x == 0)
        return 1;

#ifndef _WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
      if (isPowerOfTwo(_x))
        return _x;
#ifndef _WIN32
#pragma GCC diagnostic pop
#endif

      while (_x & (_x - 1))
        _x = _x & (_x - 1);

      _x = _x << 1;

      return _x;
    }

    /// \brief parse string into an integer
    /// \param[in] _input the string
    /// \return an integer, 0 or 0 and a message in the error stream
    /// \deprecated See ignition::math::parseInt
    inline int
    GAZEBO_DEPRECATED(8.0)
    parseInt(const std::string& _input)
    {
      const char *p = _input.c_str();
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

      int acc = 0;
      while (*p >= '0' && *p <= '9')
        acc = acc * 10 + *p++ - '0';

      if (*p)
      {
        std::cerr << "Invalid int numeric format[" << _input << "]\n";
        return 0.0;
      }

      return s * acc;
    }

    /// \brief parse string into float
    /// \param _input the string
    /// \return a floating point number (can be NaN) or 0 with a message in the
    /// error stream
    /// \deprecated See ignition::math::parseFloat
    inline double
    GAZEBO_DEPRECATED(8.0)
    parseFloat(const std::string& _input)
    {
      const char *p = _input.c_str();
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
        std::cerr << "Invalid double numeric format[" << _input << "]\n";
        return 0.0;
      }
      return s * acc;
    }
    /// \}
  }
}
#endif
