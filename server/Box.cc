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
#include "Box.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Default constructor
Box::Box()
{
}


////////////////////////////////////////////////////////////////////////////////
/// Constructor
Box::Box (const Vector3 min, const Vector3 max)
  : min(min), max(max)
{
}

////////////////////////////////////////////////////////////////////////////////
/// Copy Constructor
Box::Box( const Box &b )
  : min(b.min), max(b.max)
{
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
Box::~Box()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Get the length along the x dimension
double Box::GetXLength()
{
  return fabs(max.x - min.x);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the length along the y dimension
double Box::GetYLength()
{
  return fabs(max.y - min.y);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the length along the z dimension
double Box::GetZLength()
{
  return fabs(max.z - min.z);
}

////////////////////////////////////////////////////////////////////////////////
/// Merge a box with this box
void Box::Merge(const Box &box)
{
  this->min.SetToMin(box.min);
  this->max.SetToMax(box.max);
}

////////////////////////////////////////////////////////////////////////////////
/// Equal operator
const Box &Box::operator=( const Box &b )
{
  this->max = b.max;
  this->min = b.min;

  return *this;
}
