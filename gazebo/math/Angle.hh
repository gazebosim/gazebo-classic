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
/* Desc: Angle class
 * Author: Nate Koenig
 * Date: 18 Aug 2008
 */

#ifndef ANGLE_HH
#define ANGLE_HH

#include <math.h>
#include <iostream>

// Convert radians to degrees
#define GZ_RTOD(r) ((r) * 180 / M_PI)

// Convert degrees to radians
#define GZ_DTOR(d) ((d) * M_PI / 180)

// Normalize an angle in the range -Pi to Pi
#define GZ_NORMALIZE(a) (atan2(sin(a), cos(a)))

namespace gazebo
{
  /// \ingroup gazebo_math

  /// \brief Math namespace
  namespace math
  {
  /// \addtogroup gazebo_math Math
  /// \{

  /// \brief An angle and related functions.
  class Angle
  {
    /// \brief Constructor
    public: Angle();

    /// \brief Copy Constructor
    /// \param _radian Radians
    public: Angle(double _radian);

    /// \brief Copy constructor
    /// \param _angle Angle to copy
    public: Angle(const Angle &_angle);

    /// \brief Destructor
    public: virtual ~Angle();

    /// \brief Set the value from an angle in radians
    /// \param _radian Radian value
    public: void SetFromRadian(double _radian);

    /// \brief Set the value from an angle in degrees
    /// \param _degree Degree value
    public: void SetFromDegree(double _degree);

    /// \brief Get the angle in radians
    /// \return Double containing the angle's radian value
    public: double GetAsRadian() const __attribute__((deprecated));

    /// \brief Get the angle in radians
    /// \return Double containing the angle's radian value
    public: double Radian() const;

    /// \brief Get the angle in degrees
    /// \return Double containing the angle's degree value
    public: double GetAsDegree() const __attribute__((deprecated));

    /// \brief Get the angle in degrees
    /// \return Double containing the angle's degree value
    public: double Degree() const;

    /// \brief Normalize the angle
    public: void Normalize();

    /// \brief Dereference operator
    /// \return Double containing the angle's radian value
    public: inline double operator*() const { return value; }
    /// \brief Substraction, result = this - _angle
    /// \param _angle Angle for substraction
    /// \return The new angle
    public: Angle operator-(const Angle &_angle) const;

    /// \brief Addition, result = this + _angle
    /// \param _angle Angle for addition
    /// \return The new angle
    public: Angle operator+(const Angle &_angle) const;

    /// \brief Multiplication, result = this * _angle
    /// \param _angle Angle for multiplication
    /// \return The new angle
    public: Angle operator*(const Angle &_angle) const;

    /// \brief Division, result = this / _angle
    /// \param _angle Angle for division
    /// \return The new angle
    public: Angle operator/(const Angle &_angle) const;

    /// \brief Subtraction set, this = this - _angle
    /// \param _angle Angle for subtraction
    /// \return This angle
    public: Angle operator-=(const Angle &_angle);

    /// \brief Addition set, this = this + _angle
    /// \param _angle Angle for addition
    /// \return This angle
    public: Angle operator+=(const Angle &_angle);

    /// \brief Multiplication set, this = this * _angle
    /// \param _angle Angle for multiplication
    /// \return This angle
    public: Angle operator*=(const Angle &_angle);

    /// \brief Division set, this = this / _angle
    /// \param _angle Angle for division
    /// \return This angle
    public: Angle operator/=(const Angle &_angle);

    /// \brief Equality operator, result = this == _angle
    /// \param _angle Angle to check for equality
    /// \return True if this == _angle
    public: bool operator ==(const Angle &_angle) const;

    /// \brief Inequality
    /// \param _angle Angle to check for inequality
    /// \return True if this != _angle
    public: bool operator!=(const Angle &_angle) const;

    /// \brief Less than operator
    /// \param _angle Angle to check
    /// \return True if this < _angle
    public: bool operator<(const Angle &_angle) const;

    /// \brief Less or equal operator
    /// \param _angle Angle to check
    /// \return True if this <= _angle
    public: bool operator<=(const Angle &_angle) const;

    /// \brief Greater than operator
    /// \param _angle Angle to check
    /// \return True if this > _angle
    public: bool operator>(const Angle &_angle) const;

    /// \brief Greater equal
    /// \param _angle Angle to check
    /// \return True if this >= _angle
    public: bool operator>=(const Angle &_angle) const;

    /// \brief Ostream operator. Outputs in degrees
    /// \param out Ostream
    /// \param pt Angle to output
    /// \return The Ostream
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const gazebo::math::Angle &_a)
    {
      _out << _a.Radian();
      return _out;
    }

    /// \brief Istream operator. Assumes input is in degrees
    /// \param in Ostream
    /// \param pt Angle to read value into
    /// \return The istream
    public: friend std::istream &operator>>(std::istream &_in,
                                            gazebo::math::Angle &_a)
    {
      // Skip white spaces
      _in.setf(std::ios_base::skipws);
      _in >> _a.value;
      return _in;
    }

    /// The angle in radians
    private: double value;
  };

  /// \}
  }
}

#endif



