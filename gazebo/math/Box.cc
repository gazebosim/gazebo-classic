/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include "gazebo/math/Box.hh"

using namespace gazebo;
using namespace math;

//////////////////////////////////////////////////
Box::Box()
{
  this->min.Set(0, 0, 0);
  this->max.Set(0, 0, 0);
  this->extent = EXTENT_NULL;
}


//////////////////////////////////////////////////
Box::Box(const Vector3 &_min, const Vector3 &_max)
  : min(_min), max(_max), extent(EXTENT_FINITE)
{
}

//////////////////////////////////////////////////
Box::Box(const Box &_b)
  : min(_b.min), max(_b.max), extent(_b.extent)
{
}

//////////////////////////////////////////////////
Box::~Box()
{
}

//////////////////////////////////////////////////
double Box::GetXLength() const
{
  return fabs(max.x - min.x);
}

//////////////////////////////////////////////////
double Box::GetYLength() const
{
  return fabs(max.y - min.y);
}

//////////////////////////////////////////////////
double Box::GetZLength() const
{
  return fabs(max.z - min.z);
}

//////////////////////////////////////////////////
math::Vector3 Box::GetSize() const
{
  return math::Vector3(this->GetXLength(),
                       this->GetYLength(),
                       this->GetZLength());
}

//////////////////////////////////////////////////
math::Vector3 Box::GetCenter() const
{
  Vector3 size = this->GetSize();
  size /= 2.0;
  return this->min + size;
}


//////////////////////////////////////////////////
void Box::Merge(const Box &_box)
{
  if (this->extent == EXTENT_NULL)
  {
    this->min = _box.min;
    this->max = _box.max;
    this->extent = _box.extent;
  }
  else
  {
    this->min.SetToMin(_box.min);
    this->max.SetToMax(_box.max);
  }
}

//////////////////////////////////////////////////
Box &Box::operator =(const Box &_b)
{
  this->max = _b.max;
  this->min = _b.min;
  this->extent = _b.extent;

  return *this;
}

//////////////////////////////////////////////////
Box Box::operator+(const Box &_b) const
{
  Vector3 mn, mx;

  if (this->extent != EXTENT_NULL)
  {
    mn = this->min;
    mx = this->max;

    mn.SetToMin(_b.min);
    mx.SetToMax(_b.max);
  }
  else
  {
    mn = _b.min;
    mx = _b.max;
  }

  return Box(mn, mx);
}

//////////////////////////////////////////////////
const Box &Box::operator+=(const Box &_b)
{
  if (this->extent != EXTENT_NULL)
  {
    this->min.SetToMin(_b.min);
    this->max.SetToMax(_b.max);
  }
  else
  {
    this->min = _b.min;
    this->max = _b.max;
    this->extent = _b.extent;
  }
  return *this;
}

//////////////////////////////////////////////////
bool Box::operator==(const Box &_b)
{
  return this->min == _b.min && this->max == _b.max;
}

//////////////////////////////////////////////////
Box Box::operator-(const Vector3 &_v)
{
  return Box(this->min - _v, this->max - _v);
}

//////////////////////////////////////////////////
bool Box::Intersects(const Box &_box) const
{
  // Check the six separating planes.
  if (this->max.x < _box.min.x)
    return false;
  if (this->max.y < _box.min.y)
    return false;
  if (this->max.z < _box.min.z)
    return false;

  if (this->min.x > _box.max.x)
    return false;
  if (this->min.y > _box.max.y)
    return false;
  if (this->min.z > _box.max.z)
    return false;

  // Otherwise the two boxes must intersect
  return true;
}

//////////////////////////////////////////////////
bool Box::Inside(const Vector3 &_pt) const
{
  return this->Inside(_pt.x, _pt.y, _pt.z);
}

//////////////////////////////////////////////////
bool Box::Inside(double _x, double _y, double _z) const
{
  return _x >= this->min.x && _x <= this->max.x &&
         _y >= this->min.y && _y <= this->max.y &&
         _z >= this->min.z && _z <= this->max.z;
  return true;
}
