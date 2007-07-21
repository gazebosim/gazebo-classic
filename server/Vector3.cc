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
/* Desc: Vector 3
 * Author: Andrew Howard and Nate Koenig
 * Date: 4 Apr 2007
 * SVN: $Id$
 */

#include <math.h>

#include "Vector3.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Vector3::Vector3()
  : x(0), y(0), z(0)
{
}

////////////////////////////////////////////////////////////////////////////////
// Constructor
Vector3::Vector3( const double &x, const double &y, const double &z )
  : x(x), y(y), z(z)
{
}

////////////////////////////////////////////////////////////////////////////////
// Copy Constructor
Vector3::Vector3( const Vector3 &pt )
  : x(pt.x), y(pt.y), z(pt.z)
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Vector3::~Vector3()
{
}

////////////////////////////////////////////////////////////////////////////////
// Calc distance to the given point
double Vector3::Distance(const Vector3 &pt ) const
{
  return sqrt((this->x-pt.x)*(this->x-pt.x) + (this->y-pt.y)*(this->y-pt.y) + (this->z-pt.z)*(this->z-pt.z));
}

////////////////////////////////////////////////////////////////////////////////
// Returns the length (magnitude) of the vector
double Vector3::GetLength() const
{
  return sqrt(this->x * this->x + this->y * this->y + this->z * this->z);
}

////////////////////////////////////////////////////////////////////////////////
// Return the square of the length (magnitude) of the vector
double Vector3::GetSquaredLength() const
{
  return this->x * this->x + this->y * this->y + this->z * this->z;
}

////////////////////////////////////////////////////////////////////////////////
// Normalize the vector length
void Vector3::Normalize()
{
  double d = sqrt(this->x * this->x + this->y * this->y + this->z * this->z);

  this->x /= d;
  this->y /= d;
  this->z /= d;
}

////////////////////////////////////////////////////////////////////////////////
// Set the contents of the vector
void Vector3::Set(double x, double y, double z)
{
  this->x = x;
  this->y = y;
  this->z = z;
}

////////////////////////////////////////////////////////////////////////////////
Vector3 Vector3::GetCrossProd(const Vector3 &pt) const
{
  Vector3 c;

  c.x =  this->y * pt.z - this->z * pt.y;
  c.y = -this->x * pt.z + this->z * pt.x;
  c.z =  this->x * pt.y - this->y * pt.x;

  return c;
}

////////////////////////////////////////////////////////////////////////////////
/// Return a vector that is perpendicular to this one.
Vector3 Vector3::GetPerpendicular() const
{
  static const double sqrZero = 1e-06 * 1e-06;

  Vector3 perp = this->GetCrossProd( Vector3(1,0,0) );

  // Check the length of the vector
  if (perp.GetSquaredLength() < sqrZero)
  {
    perp = this->GetCrossProd( Vector3(0,1,0) );
  }

  return perp;
}


////////////////////////////////////////////////////////////////////////////////
// Equals operator
const Vector3 &Vector3::operator=( const Vector3 &pt )
{
  this->x = pt.x;
  this->y = pt.y;
  this->z = pt.z;

  return *this;
}

////////////////////////////////////////////////////////////////////////////////
/// Equal operator
const Vector3 &Vector3::operator=( double value )
{
  this->x = value;
  this->y = value; 
  this->z = value;

  return *this;
}



////////////////////////////////////////////////////////////////////////////////
// Addition operator
Vector3 Vector3::operator+( const Vector3 &pt ) const
{
  return Vector3(this->x + pt.x, this->y + pt.y, this->z + pt.z);
}

const Vector3 &Vector3::operator+=( const Vector3 &pt )
{
  this->x += pt.x;
  this->y += pt.y;
  this->z += pt.z;

  return *this;
}

////////////////////////////////////////////////////////////////////////////////
// Subtraction operators
Vector3 Vector3::operator-( const Vector3 &pt ) const
{
  return Vector3(this->x - pt.x, this->y - pt.y, this->z - pt.z);
}

const Vector3 &Vector3::operator-=( const Vector3 &pt )
{
  this->x -= pt.x;
  this->y -= pt.y;
  this->z -= pt.z;

  return *this;
}


////////////////////////////////////////////////////////////////////////////////
// Division operators

const Vector3 Vector3::operator/( const Vector3 &pt ) const
{
  return Vector3(this->x / pt.x, this->y / pt.y, this->z / pt.z);
}

const Vector3 &Vector3::operator/=( const Vector3 &pt )
{
  this->x /= pt.x;
  this->y /= pt.y;
  this->z /= pt.z;

  return *this;
}

const Vector3 Vector3::operator/( double v ) const
{
  return Vector3(this->x / v, this->y / v, this->z / v);
}

const Vector3 &Vector3::operator/=( double v )
{
  this->x /= v;
  this->y /= v;
  this->z /= v;

  return *this;
}



////////////////////////////////////////////////////////////////////////////////
// Mulitplication operators
const Vector3 Vector3::operator*( const Vector3 &pt ) const
{
  return Vector3(this->x * pt.x, this->y * pt.y, this->z * pt.z);
}

const Vector3 &Vector3::operator*=( const Vector3 &pt )
{
  this->x *= pt.x;
  this->y *= pt.y;
  this->z *= pt.z;

  return *this;
}

const Vector3 Vector3::operator*( double v ) const
{
  return Vector3(this->x * v, this->y * v, this->z * v);
}

const Vector3 &Vector3::operator*=( double v)
{
  this->x *= v;
  this->y *= v;
  this->z *= v;

  return *this;
}

////////////////////////////////////////////////////////////////////////////////
// Equality operator
bool Vector3::operator==( const Vector3 &pt ) const
{
  return this->x == pt.x && this->y == pt.y && this->z == pt.z;
}

////////////////////////////////////////////////////////////////////////////////
// Inequality operator
bool Vector3::operator!=( const Vector3 &pt ) const
{
  return !(*this == pt);
}

////////////////////////////////////////////////////////////////////////////////
// See if a point is finite (e.g., not nan)
bool Vector3::IsFinite() const
{
  return finite(this->x) && finite(this->y) && finite(this->z);
}

////////////////////////////////////////////////////////////////////////////////
/// [] operator
double Vector3::operator[](unsigned int index) const
{
  switch (index)
  {
    case 0:
      return this->x;
    case 1:
      return this->y;
    case 2:
      return this->z;
    default:
      return 0;
  }
}
