/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
 * SVN: $Id$
 */

#include <math.h>
#include "math/Angle.hh"

using namespace gazebo;
using namespace math;

//////////////////////////////////////////////////
/// Constructor
Angle::Angle()
{
  this->value = 0;
}

//////////////////////////////////////////////////
// Constructor
Angle::Angle(double _radian)
{
  this->value = _radian;
}

//////////////////////////////////////////////////
/// Copy constructor
Angle::Angle(const Angle &_angle)
{
  this->value = _angle.value;
}

//////////////////////////////////////////////////
/// Destructor
Angle::~Angle()
{
}

//////////////////////////////////////////////////
/// Set the value from an angle in radians
void Angle::SetFromRadian(double _radian)
{
  this->value = _radian;
}

//////////////////////////////////////////////////
/// Set the value from an angle in degrees
void Angle::SetFromDegree(double _degree)
{
  this->value = _degree * M_PI / 180.0;
}

//////////////////////////////////////////////////
/// Get the angle in radians
double Angle::GetAsRadian() const
{
  return this->value;
}

//////////////////////////////////////////////////
/// Get the angle in degrees
double Angle::GetAsDegree() const
{
  return this->value * 180.0 / M_PI;
}

//////////////////////////////////////////////////
// Normalize the angle
void Angle::Normalize()
{
  this->value = atan2(sin(this->value), cos(this->value));
}

//////////////////////////////////////////////////
/// Substraction
Angle Angle::operator-(const Angle &angle) const
{
  return Angle(this->value - angle.value);
}

//////////////////////////////////////////////////
/// Addition
Angle Angle::operator+(const Angle &angle) const
{
  return Angle(this->value + angle.value);
}

//////////////////////////////////////////////////
/// Multiplication
Angle Angle::operator*(const Angle &angle) const
{
  return Angle(this->value * angle.value);
}

//////////////////////////////////////////////////
/// Division
Angle Angle::operator/(const Angle &angle) const
{
  return Angle(this->value / angle.value);
}

//////////////////////////////////////////////////
/// Add set
Angle Angle::operator-=(const Angle &angle)
{
  this->value -= angle.value;
  return *this;
}

//////////////////////////////////////////////////
/// Sub set
Angle Angle::operator+=(const Angle &angle)
{
  this->value += angle.value;
  return *this;
}

//////////////////////////////////////////////////
/// Mul set
Angle Angle::operator*=(const Angle &angle)
{
  this->value *= angle.value;
  return *this;
}

//////////////////////////////////////////////////
/// Div set
Angle Angle::operator/=(const Angle &angle)
{
  this->value /= angle.value;
  return *this;
}

//////////////////////////////////////////////////
/// Equality
bool Angle::operator ==(const Angle &angle) const
{
  return this->value == angle.value;
}

//////////////////////////////////////////////////
/// Inequality
bool Angle::operator!=(const Angle &angle) const
{
  return !(*this == angle);
}

//////////////////////////////////////////////////
/// Less
bool Angle::operator<(const Angle &angle) const
{
  return this->value < angle.value;
}

//////////////////////////////////////////////////
/// Less equal
bool Angle::operator<=(const Angle &angle) const
{
  return this->value <= angle.value;
}

//////////////////////////////////////////////////
/// Greater
bool Angle::operator>(const Angle &angle) const
{
  return this->value > angle.value;
}

//////////////////////////////////////////////////
/// Greater equal
bool Angle::operator>=(const Angle &angle) const
{
  return this->value >= angle.value;
}


