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
 */

#include <math.h>
#include "math/Vector2d.hh"

using namespace gazebo;
using namespace math;

//////////////////////////////////////////////////
// Constructor
Vector2d::Vector2d()
  : x(0), y(0)
{
}

//////////////////////////////////////////////////
// Constructor
Vector2d::Vector2d(const double &_x, const double &_y)
  : x(_x), y(_y)
{
}

//////////////////////////////////////////////////
// Copy Constructor
Vector2d::Vector2d(const Vector2d &_pt)
  : x(_pt.x), y(_pt.y)
{
}

//////////////////////////////////////////////////
// Destructor
Vector2d::~Vector2d()
{
}

//////////////////////////////////////////////////
// Calc distance to the given point
double Vector2d::Distance(const Vector2d &_pt) const
{
  return sqrt((this->x-_pt.x)*(this->x-_pt.x) +
              (this->y-_pt.y)*(this->y-_pt.y));
}

//////////////////////////////////////////////////
// Normalize the vector length
void Vector2d::Normalize()
{
  double d = sqrt(this->x * this->x + this->y * this->y);

  this->x /= d;
  this->y /= d;
}

//////////////////////////////////////////////////
// Set the contents of the vector
void Vector2d::Set(double _x, double _y)
{
  this->x = _x;
  this->y = _y;
}


//////////////////////////////////////////////////
// Equals operator
const Vector2d &Vector2d::operator =(const Vector2d &pt)
{
  this->x = pt.x;
  this->y = pt.y;

  return *this;
}

//////////////////////////////////////////////////
/// Equal operator
const Vector2d &Vector2d::operator =(double value)
{
  this->x = value;
  this->y = value;

  return *this;
}



//////////////////////////////////////////////////
// Addition operator
Vector2d Vector2d::operator+(const Vector2d &pt) const
{
  return Vector2d(this->x + pt.x, this->y + pt.y);
}

const Vector2d &Vector2d::operator+=(const Vector2d &pt)
{
  this->x += pt.x;
  this->y += pt.y;

  return *this;
}

//////////////////////////////////////////////////
// Subtraction operators
Vector2d Vector2d::operator-(const Vector2d &pt) const
{
  return Vector2d(this->x - pt.x, this->y - pt.y);
}

const Vector2d &Vector2d::operator-=(const Vector2d &pt)
{
  this->x -= pt.x;
  this->y -= pt.y;

  return *this;
}


//////////////////////////////////////////////////
// Division operators

const Vector2d Vector2d::operator/(const Vector2d &pt) const
{
  return Vector2d(this->x / pt.x, this->y / pt.y);
}

const Vector2d &Vector2d::operator/=(const Vector2d &pt)
{
  this->x /= pt.x;
  this->y /= pt.y;

  return *this;
}

const Vector2d Vector2d::operator/(double v) const
{
  return Vector2d(this->x / v, this->y / v);
}

const Vector2d &Vector2d::operator/=(double v)
{
  this->x /= v;
  this->y /= v;

  return *this;
}



//////////////////////////////////////////////////
// Mulitplication operators
const Vector2d Vector2d::operator*(const Vector2d &pt) const
{
  return Vector2d(this->x * pt.x, this->y * pt.y);
}

const Vector2d &Vector2d::operator*=(const Vector2d &pt)
{
  this->x *= pt.x;
  this->y *= pt.y;

  return *this;
}

const Vector2d Vector2d::operator*(double v) const
{
  return Vector2d(this->x * v, this->y * v);
}

const Vector2d &Vector2d::operator*=(double v)
{
  this->x *= v;
  this->y *= v;

  return *this;
}

//////////////////////////////////////////////////
// Equality operator
bool Vector2d::operator ==(const Vector2d &pt) const
{
  return this->x == pt.x && this->y == pt.y;
}

//////////////////////////////////////////////////
// Inequality operator
bool Vector2d::IsFinite() const
{
  return finite(this->x) && finite(this->y);
}

//////////////////////////////////////////////////
/// [] operator
double Vector2d::operator[](unsigned int index) const
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

