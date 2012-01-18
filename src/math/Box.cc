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
#include <math.h>
#include <float.h>
#include "math/Box.hh"

using namespace gazebo;
using namespace math;

//////////////////////////////////////////////////
/// Default constructor
Box::Box()
{
  this->min.Set(FLT_MAX, FLT_MAX, FLT_MAX);
  this->max.Set(-FLT_MAX, -FLT_MAX, -FLT_MAX);
}


//////////////////////////////////////////////////
/// Constructor
Box::Box(const Vector3 &_min, const Vector3 &_max)
  : min(_min), max(_max)
{
}

//////////////////////////////////////////////////
/// Copy Constructor
Box::Box(const Box &_b)
  : min(_b.min), max(_b.max)
{
}

//////////////////////////////////////////////////
/// Destructor
Box::~Box()
{
}

//////////////////////////////////////////////////
/// Get the length along the x dimension
double Box::GetXLength() const
{
  return fabs(max.x - min.x);
}

//////////////////////////////////////////////////
/// Get the length along the y dimension
double Box::GetYLength() const
{
  return fabs(max.y - min.y);
}

//////////////////////////////////////////////////
/// Get the length along the z dimension
double Box::GetZLength() const
{
  return fabs(max.z - min.z);
}

//////////////////////////////////////////////////
/// Get the size of the box
math::Vector3 Box::GetSize() const
{
  return math::Vector3(this->GetXLength(),
                       this->GetYLength(),
                       this->GetZLength());
}

//////////////////////////////////////////////////
/// Get the box center
math::Vector3 Box::GetCenter() const
{
  Vector3 size = this->GetSize();
  size /= 2.0;
  return this->min + size;
}


//////////////////////////////////////////////////
/// Merge a box with this box
void Box::Merge(const Box &_box)
{
  this->min.SetToMin(_box.min);
  this->max.SetToMax(_box.max);
}

//////////////////////////////////////////////////
/// Equal operator
Box &Box::operator =(const Box &b)
{
  this->max = b.max;
  this->min = b.min;

  return *this;
}

//////////////////////////////////////////////////
Box Box::operator+(const Box &b) const
{
  Vector3 mn = this->min;
  Vector3 mx = this->max;

  mn.SetToMin(b.min);
  mx.SetToMax(b.max);

  return Box(mn, mx);
}

//////////////////////////////////////////////////
const Box &Box::operator+=(const Box &b)
{
  this->min.SetToMin(b.min);
  this->max.SetToMax(b.max);
  return *this;
}


