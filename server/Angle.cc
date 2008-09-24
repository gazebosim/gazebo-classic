/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: Angle class
 * Author: Nate Koenig
 * Date: 18 Aug 2008
 * SVN: $Id$
 */

#include <math.h>
#include "Angle.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
Angle::Angle()
{
  this->value = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Constructor
Angle::Angle(double radian)
{
  this->value = radian;
}

////////////////////////////////////////////////////////////////////////////////
/// Copy constructor
Angle::Angle(const Angle &angle)
{
  this->value = angle.value;
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
Angle::~Angle()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Set the value from an angle in radians
void Angle::SetFromRadian( double radian )
{
  this->value = radian;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the value from an angle in degrees
void Angle::SetFromDegree( double degree )
{
  this->value = degree * M_PI / 180.0;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angle in radians
double Angle::GetAsRadian() const
{
  return this->value;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angle in degrees
double Angle::GetAsDegree() const
{
  return this->value * 180.0 / M_PI;
}

////////////////////////////////////////////////////////////////////////////////
// Normalize the angle
void Angle::Normalize()
{
 this->value = atan2(sin(this->value), cos(this->value));
}

////////////////////////////////////////////////////////////////////////////////
/// Substraction
Angle Angle::operator-(const Angle &angle) const
{
  return Angle(this->value - angle.value);
}

////////////////////////////////////////////////////////////////////////////////
/// Addition
Angle Angle::operator+(const Angle &angle) const
{
  return Angle(this->value + angle.value);
}

////////////////////////////////////////////////////////////////////////////////
/// Multiplication
Angle Angle::operator*(const Angle &angle) const
{
  return Angle(this->value * angle.value);
}

////////////////////////////////////////////////////////////////////////////////
/// Division
Angle Angle::operator/(const Angle &angle) const
{
  return Angle(this->value / angle.value);
}

////////////////////////////////////////////////////////////////////////////////
/// Add set
Angle Angle::operator-=(const Angle &angle)
{
  this->value -= angle.value;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
/// Sub set
Angle Angle::operator+=(const Angle &angle)
{
  this->value += angle.value;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
/// Mul set
Angle Angle::operator*=(const Angle &angle)
{
  this->value *= angle.value;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
/// Div set
Angle Angle::operator/=(const Angle &angle)
{
  this->value /= angle.value;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
/// Equality
bool Angle::operator==(const Angle &angle) const
{
  return this->value == angle.value;
}

////////////////////////////////////////////////////////////////////////////////
/// Inequality
bool Angle::operator!=(const Angle &angle) const
{
  return !(*this == angle);
}

////////////////////////////////////////////////////////////////////////////////
/// Less
bool Angle::operator<(const Angle &angle) const
{
  return this->value < angle.value;
}

////////////////////////////////////////////////////////////////////////////////
/// Less equal
bool Angle::operator<=(const Angle &angle) const
{
  return this->value <= angle.value;
}

////////////////////////////////////////////////////////////////////////////////
/// Greater
bool Angle::operator>(const Angle &angle) const
{
  return this->value > angle.value;
}

////////////////////////////////////////////////////////////////////////////////
/// Greater equal
bool Angle::operator>=(const Angle &angle) const
{
  return this->value >= angle.value;
}
