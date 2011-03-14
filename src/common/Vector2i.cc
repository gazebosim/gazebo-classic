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
/* Desc: Vector 2
 * Author: Nate Koenig
 * Date: 21 July 2007
 * SVN: $Id$
 */

#include <math.h>
#include "common/Vector2i.hh"

using namespace gazebo;
using namespace common;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Vector2i::Vector2i()
  : x(0), y(0)
{
}

////////////////////////////////////////////////////////////////////////////////
// Constructor
Vector2i::Vector2i( const int &x, const int &y )
  : x(x), y(y)
{
}

////////////////////////////////////////////////////////////////////////////////
// Copy Constructor
Vector2i::Vector2i( const Vector2i &pt )
  : x(pt.x), y(pt.y)
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Vector2i::~Vector2i()
{
}

////////////////////////////////////////////////////////////////////////////////
// Calc distance to the given point
int Vector2i::Distance(const Vector2i &pt ) const
{
  return (T)sqrt((this->x-pt.x)*(this->x-pt.x) + (this->y-pt.y)*(this->y-pt.y));
}

////////////////////////////////////////////////////////////////////////////////
// Normalize the vector length
void Vector2i::Normalize()
{
  int d = (T)sqrt(this->x * this->x + this->y * this->y);

  this->x /= d;
  this->y /= d;
}

////////////////////////////////////////////////////////////////////////////////
// Set the contents of the vector
void Vector2i::Set(int x, int y)
{
  this->x = x;
  this->y = y;
}


////////////////////////////////////////////////////////////////////////////////
// Equals operator
const Vector2i &Vector2i::operator=( const Vector2i &pt )
{
  this->x = pt.x;
  this->y = pt.y;

  return *this;
}

////////////////////////////////////////////////////////////////////////////////
/// Equal operator
const Vector2i &Vector2i::operator=( int value )
{
  this->x = value;
  this->y = value; 

  return *this;
}



////////////////////////////////////////////////////////////////////////////////
// Addition operator
Vector2i Vector2i::operator+( const Vector2i &pt ) const
{
  return Vector2i(this->x + pt.x, this->y + pt.y);
}

const Vector2i &Vector2i::operator+=( const Vector2i &pt )
{
  this->x += pt.x;
  this->y += pt.y;

  return *this;
}

////////////////////////////////////////////////////////////////////////////////
// Subtraction operators
Vector2i Vector2i::operator-( const Vector2i &pt ) const
{
  return Vector2i(this->x - pt.x, this->y - pt.y);
}

const Vector2i &Vector2i::operator-=( const Vector2i &pt )
{
  this->x -= pt.x;
  this->y -= pt.y;

  return *this;
}


////////////////////////////////////////////////////////////////////////////////
// Division operators

const Vector2i Vector2i::operator/( const Vector2i &pt ) const
{
  return Vector2i(this->x / pt.x, this->y / pt.y);
}

const Vector2i &Vector2i::operator/=( const Vector2i &pt )
{
  this->x /= pt.x;
  this->y /= pt.y;

  return *this;
}

const Vector2i Vector2i::operator/( int v ) const
{
  return Vector2i(this->x / v, this->y / v);
}

const Vector2i &Vector2i::operator/=( int v )
{
  this->x /= v;
  this->y /= v;

  return *this;
}



////////////////////////////////////////////////////////////////////////////////
// Mulitplication operators
const Vector2i Vector2i::operator*( const Vector2i &pt ) const
{
  return Vector2i(this->x * pt.x, this->y * pt.y);
}

const Vector2i &Vector2i::operator*=( const Vector2i &pt )
{
  this->x *= pt.x;
  this->y *= pt.y;

  return *this;
}

const Vector2i Vector2i::operator*( int v ) const
{
  return Vector2i(this->x * v, this->y * v);
}

const Vector2i &Vector2i::operator*=( int v)
{
  this->x *= v;
  this->y *= v;

  return *this;
}

////////////////////////////////////////////////////////////////////////////////
// Equality operator
bool Vector2i::operator==( const Vector2i &pt ) const
{
  return this->x == pt.x && this->y == pt.y;
}

////////////////////////////////////////////////////////////////////////////////
// Inequality operator
bool Vector2i::IsFinite() const
{
  return finite(this->x) && finite(this->y);
}

////////////////////////////////////////////////////////////////////////////////
/// [] operator
int Vector2i::operator[](unsigned int index) const
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
