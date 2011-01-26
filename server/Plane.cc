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
#include "Plane.hh"

using namespace gazebo;

Plane::Plane()
{
  this->d = 0.0;
}

Plane::~Plane()
{
}

void Plane::Set(Vector3 n, Vector2<double> s, double offset)
{
  this->normal = n;
  this->size = s;
  this->d = offset;
}

////////////////////////////////////////////////////////////////////////////////
/// Equal operator
const Plane &Plane::operator=(const Plane & p)
{
  this->normal = p.normal;
  this->size = p.size;
  this->d = p.d;

  return *this;
}
