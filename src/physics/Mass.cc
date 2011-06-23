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
/* Desc: Mass class
 * Author: Nate Koenig
 * Date: 18 May 2009
 * SVN: $Id:$
 */

#include "physics/Mass.hh"

using namespace gazebo;
using namespace physics;


////////////////////////////////////////////////////////////////////////////////
/// Default Constructor
Mass::Mass()
{
  this->mass = 0;
  this->principals.Set(0,0,0);
  this->products.Set(0,0,0);
}

////////////////////////////////////////////////////////////////////////////////
/// Constructor
Mass::Mass(double m)
{
  this->mass = m;
  this->cog.Set(0,0,0);
  this->principals.Set(0,0,0);
  this->products.Set(0,0,0);
}

////////////////////////////////////////////////////////////////////////////////
/// Copy constructor
Mass::Mass(const Mass &mass)
{
  (*this) = mass;
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
Mass::~Mass()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Reset all the mass properties
void Mass::Reset()
{
  this->mass = 0;
  this->cog.Set(0,0,0);
  this->principals.Set(0,0,0);
  this->products.Set(0,0,0);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the mass
void Mass::SetMass(double m)
{
  this->mass = m;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the mass value
double Mass::GetAsDouble() const
{
  return this->mass;
}

////////////////////////////////////////////////////////////////////////////////
// Set the center of gravity
void Mass::SetCoG(double cx, double cy, double cz)
{
  this->cog.Set(cx, cy, cz);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the center of gravity
void Mass::SetCoG(const math::Vector3 &c)
{
  this->cog = c;
}

////////////////////////////////////////////////////////////////////////////////
// Get the center of gravity
math::Vector3 Mass::GetCoG() const
{
  return this->cog;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the mass matrix
void Mass::SetInertiaMatrix(double ixx, double iyy, double izz,
                            double ixy, double ixz, double iyz)
{
  this->principals.Set( ixx, iyy, izz );
  this->products.Set( ixy, ixz, iyz );
}


////////////////////////////////////////////////////////////////////////////////
/// Get the prinicpal moments of inertia (Ixx, Iyy, Izz)
math::Vector3 Mass::GetPrincipalMoments() const
{
  return this->principals;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the products of inertia (Ixy, Ixy, Iyz)
math::Vector3 Mass::GetProductsofInertia() const
{
  return this->products;
}

////////////////////////////////////////////////////////////////////////////////
/// Rotate this mass
void Mass::Rotate(const math::Quatern &rot)
{
  this->cog = rot.RotateVector(this->cog);
}

////////////////////////////////////////////////////////////////////////////////
/// Equal operator
void Mass::operator=(const Mass &mass)
{
  this->mass = mass.mass;
  this->cog = mass.cog;
  this->principals = mass.principals;
  this->products = mass.products;
}

Mass Mass::operator+(const Mass &_mass ) const
{
  Mass result;
  result.mass = this->mass + _mass.mass;

  result.cog = (this->cog*this->mass + _mass.cog * _mass.mass) / result.mass;

  result.principals = this->principals + _mass.principals;
  result.products = this->products + _mass.products;
  return result;
}

const Mass &Mass::operator+=(const Mass &_mass )
{
  *this = *this + _mass;
  return *this;
}

