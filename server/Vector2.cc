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
/* Desc: Vector 2
 * Author: Nate Koenig
 * Date: 21 July 2007
 * SVN: $Id:$
 */

#include <math.h>

#include "Vector2.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Vector2::Vector2()
  : x(0), y(0)
{
}

////////////////////////////////////////////////////////////////////////////////
// Constructor
Vector2::Vector2( const double &x, const double &y )
  : x(x), y(y)
{
}

////////////////////////////////////////////////////////////////////////////////
// Copy Constructor
Vector2::Vector2( const Vector2 &pt )
  : x(pt.x), y(pt.y)
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Vector2::~Vector2()
{
}

////////////////////////////////////////////////////////////////////////////////
// Calc distance to the given point
double Vector2::Distance(const Vector2 &pt ) const
{
  return sqrt((this->x-pt.x)*(this->x-pt.x) + (this->y-pt.y)*(this->y-pt.y));
}

////////////////////////////////////////////////////////////////////////////////
// Normalize the vector length
void Vector2::Normalize()
{
  double d = sqrt(this->x * this->x + this->y * this->y);

  this->x /= d;
  this->y /= d;
}

////////////////////////////////////////////////////////////////////////////////
// Set the contents of the vector
void Vector2::Set(double x, double y)
{
  this->x = x;
  this->y = y;
}


////////////////////////////////////////////////////////////////////////////////
// Equals operator
const Vector2 &Vector2::operator=( const Vector2 &pt )
{
  this->x = pt.x;
  this->y = pt.y;

  return *this;
}

////////////////////////////////////////////////////////////////////////////////
/// Equal operator
const Vector2 &Vector2::operator=( double value )
{
  this->x = value;
  this->y = value; 

  return *this;
}



////////////////////////////////////////////////////////////////////////////////
// Addition operator
Vector2 Vector2::operator+( const Vector2 &pt ) const
{
  return Vector2(this->x + pt.x, this->y + pt.y);
}

const Vector2 &Vector2::operator+=( const Vector2 &pt )
{
  this->x += pt.x;
  this->y += pt.y;

  return *this;
}

////////////////////////////////////////////////////////////////////////////////
// Subtraction operators
Vector2 Vector2::operator-( const Vector2 &pt ) const
{
  return Vector2(this->x - pt.x, this->y - pt.y);
}

const Vector2 &Vector2::operator-=( const Vector2 &pt )
{
  this->x -= pt.x;
  this->y -= pt.y;

  return *this;
}


////////////////////////////////////////////////////////////////////////////////
// Division operators

const Vector2 Vector2::operator/( const Vector2 &pt ) const
{
  return Vector2(this->x / pt.x, this->y / pt.y);
}

const Vector2 &Vector2::operator/=( const Vector2 &pt )
{
  this->x /= pt.x;
  this->y /= pt.y;

  return *this;
}

const Vector2 Vector2::operator/( double v ) const
{
  return Vector2(this->x / v, this->y / v);
}

const Vector2 &Vector2::operator/=( double v )
{
  this->x /= v;
  this->y /= v;

  return *this;
}



////////////////////////////////////////////////////////////////////////////////
// Mulitplication operators
const Vector2 Vector2::operator*( const Vector2 &pt ) const
{
  return Vector2(this->x * pt.x, this->y * pt.y);
}

const Vector2 &Vector2::operator*=( const Vector2 &pt )
{
  this->x *= pt.x;
  this->y *= pt.y;

  return *this;
}

const Vector2 Vector2::operator*( double v ) const
{
  return Vector2(this->x * v, this->y * v);
}

const Vector2 &Vector2::operator*=( double v)
{
  this->x *= v;
  this->y *= v;

  return *this;
}

////////////////////////////////////////////////////////////////////////////////
// Equality operator
bool Vector2::operator==( const Vector2 &pt ) const
{
  return this->x == pt.x && this->y == pt.y;
}

////////////////////////////////////////////////////////////////////////////////
// Inequality operator
bool Vector2::operator!=( const Vector2 &pt ) const
{
  return !(*this == pt);
}

////////////////////////////////////////////////////////////////////////////////
// See if a point is finite (e.g., not nan)
bool Vector2::IsFinite() const
{
  return finite(this->x) && finite(this->y);
}

////////////////////////////////////////////////////////////////////////////////
/// [] operator
double Vector2::operator[](unsigned int index) const
{
  switch (index)
  {
    case 0:
      return this->x;
    case 1:
      return this->y;
    default:
      return 0;
  }
}
