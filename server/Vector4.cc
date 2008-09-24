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
/* Desc: Vector 4
 * Author: Andrew Howard and Nate Koenig
 * Date: 4 Apr 2007
 * SVN: $Id$
 */

#include <math.h>

#include "Vector4.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Vector4::Vector4()
    : x(0), y(0), z(0), w(0)
{
}

////////////////////////////////////////////////////////////////////////////////
// Constructor
Vector4::Vector4( const double &x, const double &y, const double &z,const double &w )
    : x(x), y(y), z(z), w(w)
{
}

////////////////////////////////////////////////////////////////////////////////
// Copy Constructor
Vector4::Vector4( const Vector4 &pt )
    : x(pt.x), y(pt.y), z(pt.z), w(pt.w)
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Vector4::~Vector4()
{
}

////////////////////////////////////////////////////////////////////////////////
// Calc distance to the given point
double Vector4::Distance(const Vector4 &pt ) const
{
  return sqrt((this->x-pt.x)*(this->x-pt.x) + 
              (this->y-pt.y)*(this->y-pt.y) + 
              (this->z-pt.z)*(this->z-pt.z) +
              (this->w-pt.w)*(this->w-pt.w));
}

////////////////////////////////////////////////////////////////////////////////
// Returns the length (magnitude) of the vector
double Vector4::GetLength() const
{
  return sqrt(this->x * this->x + this->y * this->y + this->z * this->z + this->w * this->w);
}

////////////////////////////////////////////////////////////////////////////////
// Return the square of the length (magnitude) of the vector
double Vector4::GetSquaredLength() const
{
  return this->x * this->x + this->y * this->y + this->z * this->z + this->w * this->w;
}

////////////////////////////////////////////////////////////////////////////////
// Normalize the vector length
void Vector4::Normalize()
{
  double d = this->GetLength();

  this->x /= d;
  this->y /= d;
  this->z /= d;
  this->w /= d;
}

////////////////////////////////////////////////////////////////////////////////
// Set the contents of the vector
void Vector4::Set(double x, double y, double z, double w)
{
  this->x = x;
  this->y = y;
  this->z = z;
  this->w = w;
}


////////////////////////////////////////////////////////////////////////////////
// Equals operator
const Vector4 &Vector4::operator=( const Vector4 &pt )
{
  this->x = pt.x;
  this->y = pt.y;
  this->z = pt.z;
  this->w = pt.w;

  return *this;
}

////////////////////////////////////////////////////////////////////////////////
/// Equal operator
const Vector4 &Vector4::operator=( double value )
{
  this->x = value;
  this->y = value;
  this->z = value;
  this->w = value;

  return *this;
}



////////////////////////////////////////////////////////////////////////////////
// Addition operator
Vector4 Vector4::operator+( const Vector4 &pt ) const
{
  return Vector4(this->x + pt.x, this->y + pt.y, this->z + pt.z, this->w+pt.w);
}

const Vector4 &Vector4::operator+=( const Vector4 &pt )
{
  this->x += pt.x;
  this->y += pt.y;
  this->z += pt.z;
  this->w += pt.w;

  return *this;
}

////////////////////////////////////////////////////////////////////////////////
// Subtraction operators
Vector4 Vector4::operator-( const Vector4 &pt ) const
{
  return Vector4(this->x - pt.x, this->y - pt.y, this->z - pt.z, this->w-pt.w);
}

const Vector4 &Vector4::operator-=( const Vector4 &pt )
{
  this->x -= pt.x;
  this->y -= pt.y;
  this->z -= pt.z;
  this->w -= pt.w;

  return *this;
}


////////////////////////////////////////////////////////////////////////////////
// Division operators

const Vector4 Vector4::operator/( const Vector4 &pt ) const
{
  return Vector4(this->x / pt.x, this->y / pt.y, this->z / pt.z, this->w/pt.w);
}

const Vector4 &Vector4::operator/=( const Vector4 &pt )
{
  this->x /= pt.x;
  this->y /= pt.y;
  this->z /= pt.z;
  this->w /= pt.w;

  return *this;
}

const Vector4 Vector4::operator/( double v ) const
{
  return Vector4(this->x / v, this->y / v, this->z / v, this->w / v);
}

const Vector4 &Vector4::operator/=( double v )
{
  this->x /= v;
  this->y /= v;
  this->z /= v;
  this->w /= v;

  return *this;
}



////////////////////////////////////////////////////////////////////////////////
// Mulitplication operators
const Vector4 Vector4::operator*( const Vector4 &pt ) const
{
  return Vector4(this->x * pt.x, this->y * pt.y, this->z * pt.z, this->w*pt.w);
}

const Vector4 &Vector4::operator*=( const Vector4 &pt )
{
  this->x *= pt.x;
  this->y *= pt.y;
  this->z *= pt.z;
  this->w *= pt.w;

  return *this;
}

const Vector4 Vector4::operator*( double v ) const
{
  return Vector4(this->x * v, this->y * v, this->z * v, this->w*v);
}

const Vector4 &Vector4::operator*=( double v)
{
  this->x *= v;
  this->y *= v;
  this->z *= v;
  this->w *= v;

  return *this;
}

////////////////////////////////////////////////////////////////////////////////
// Equality operator
bool Vector4::operator==( const Vector4 &pt ) const
{
  return this->x == pt.x && this->y == pt.y && this->z == pt.z && this->w==pt.w;
}

////////////////////////////////////////////////////////////////////////////////
// Inequality operator
bool Vector4::operator!=( const Vector4 &pt ) const
{
  return !(*this == pt);
}

////////////////////////////////////////////////////////////////////////////////
// See if a point is finite (e.g., not nan)
bool Vector4::IsFinite() const
{
  return finite(this->x) && finite(this->y) && finite(this->z) && finite(this->w);
}

////////////////////////////////////////////////////////////////////////////////
/// [] operator
double Vector4::operator[](unsigned int index) const
{
  switch (index)
  {
    case 0:
      return this->x;
    case 1:
      return this->y;
    case 2:
      return this->z;
    case 3:
      return this->w;
    default:
      return 0;
  }
}
