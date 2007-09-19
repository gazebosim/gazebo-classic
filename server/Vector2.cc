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
template<typename T>
Vector2<T>::Vector2()
  : x(0), y(0)
{
}

////////////////////////////////////////////////////////////////////////////////
// Constructor
template<typename T>
Vector2<T>::Vector2( const T &x, const T &y )
  : x(x), y(y)
{
}

////////////////////////////////////////////////////////////////////////////////
// Copy Constructor
template<typename T>
Vector2<T>::Vector2( const Vector2 &pt )
  : x(pt.x), y(pt.y)
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
template<typename T>
Vector2<T>::~Vector2()
{
}

////////////////////////////////////////////////////////////////////////////////
// Calc distance to the given point
template<typename T>
T Vector2<T>::Distance(const Vector2<T> &pt ) const
{
  return (T)sqrt((this->x-pt.x)*(this->x-pt.x) + (this->y-pt.y)*(this->y-pt.y));
}

////////////////////////////////////////////////////////////////////////////////
// Normalize the vector length
template<typename T>
void Vector2<T>::Normalize()
{
  T d = (T)sqrt(this->x * this->x + this->y * this->y);

  this->x /= d;
  this->y /= d;
}

////////////////////////////////////////////////////////////////////////////////
// Set the contents of the vector
template<typename T>
void Vector2<T>::Set(T x, T y)
{
  this->x = x;
  this->y = y;
}


////////////////////////////////////////////////////////////////////////////////
// Equals operator
template<typename T>
const Vector2<T> &Vector2<T>::operator=( const Vector2<T> &pt )
{
  this->x = pt.x;
  this->y = pt.y;

  return *this;
}

////////////////////////////////////////////////////////////////////////////////
/// Equal operator
template<typename T>
const Vector2<T> &Vector2<T>::operator=( T value )
{
  this->x = value;
  this->y = value; 

  return *this;
}



////////////////////////////////////////////////////////////////////////////////
// Addition operator
template<typename T>
Vector2<T> Vector2<T>::operator+( const Vector2<T> &pt ) const
{
  return Vector2<T>(this->x + pt.x, this->y + pt.y);
}

template<typename T>
const Vector2<T> &Vector2<T>::operator+=( const Vector2<T> &pt )
{
  this->x += pt.x;
  this->y += pt.y;

  return *this;
}

////////////////////////////////////////////////////////////////////////////////
// Subtraction operators
template<typename T>
Vector2<T> Vector2<T>::operator-( const Vector2<T> &pt ) const
{
  return Vector2<T>(this->x - pt.x, this->y - pt.y);
}

template<typename T>
const Vector2<T> &Vector2<T>::operator-=( const Vector2<T> &pt )
{
  this->x -= pt.x;
  this->y -= pt.y;

  return *this;
}


////////////////////////////////////////////////////////////////////////////////
// Division operators

template<typename T>
const Vector2<T> Vector2<T>::operator/( const Vector2<T> &pt ) const
{
  return Vector2<T>(this->x / pt.x, this->y / pt.y);
}

template<typename T>
const Vector2<T> &Vector2<T>::operator/=( const Vector2<T> &pt )
{
  this->x /= pt.x;
  this->y /= pt.y;

  return *this;
}

template<typename T>
const Vector2<T> Vector2<T>::operator/( T v ) const
{
  return Vector2<T>(this->x / v, this->y / v);
}

template<typename T>
const Vector2<T> &Vector2<T>::operator/=( T v )
{
  this->x /= v;
  this->y /= v;

  return *this;
}



////////////////////////////////////////////////////////////////////////////////
// Mulitplication operators
template<typename T>
const Vector2<T> Vector2<T>::operator*( const Vector2<T> &pt ) const
{
  return Vector2<T>(this->x * pt.x, this->y * pt.y);
}

template<typename T>
const Vector2<T> &Vector2<T>::operator*=( const Vector2<T> &pt )
{
  this->x *= pt.x;
  this->y *= pt.y;

  return *this;
}

template<typename T>
const Vector2<T> Vector2<T>::operator*( T v ) const
{
  return Vector2<T>(this->x * v, this->y * v);
}

template<typename T>
const Vector2<T> &Vector2<T>::operator*=( T v)
{
  this->x *= v;
  this->y *= v;

  return *this;
}

////////////////////////////////////////////////////////////////////////////////
// Equality operator
template<typename T>
bool Vector2<T>::operator==( const Vector2<T> &pt ) const
{
  return this->x == pt.x && this->y == pt.y;
}

////////////////////////////////////////////////////////////////////////////////
// Inequality operator
template<typename T>
bool Vector2<T>::operator!=( const Vector2<T> &pt ) const
{
  return !(*this == pt);
}

////////////////////////////////////////////////////////////////////////////////
// See if a point is finite (e.g., not nan)
template<typename T>
bool Vector2<T>::IsFinite() const
{
  return finite(this->x) && finite(this->y);
}

////////////////////////////////////////////////////////////////////////////////
/// [] operator
template<typename T>
T Vector2<T>::operator[](unsigned int index) const
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
