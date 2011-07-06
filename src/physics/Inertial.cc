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
#include "Inertial.hh"

using namespace gazebo;
using namespace physics;

////////////////////////////////////////////////////////////////////////////////
/// Default Constructor
Inertial::Inertial()
{
  this->mass = 0;
  this->principals.Set(0,0,0);
  this->products.Set(0,0,0);
}

////////////////////////////////////////////////////////////////////////////////
/// Constructor
Inertial::Inertial(double m)
{
  this->mass = m;
  this->cog.Set(0,0,0);
  this->principals.Set(0,0,0);
  this->products.Set(0,0,0);
}

////////////////////////////////////////////////////////////////////////////////
/// Copy constructor
Inertial::Inertial(const Inertial &_inertial)
{
  (*this) = _inertial;
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
Inertial::~Inertial()
{
}

////////////////////////////////////////////////////////////////////////////////
void Inertial::Load( sdf::ElementPtr _sdf)
{
  this->sdf = _sdf;

  math::Vector3 center(0,0,0);
  if (this->sdf->HasElement("origin"))
    center = this->sdf->GetElement("origin")->GetValuePose("pose").pos;
  this->SetCoG(center.x, center.y, center.z);

 
  sdf::ElementPtr inertiaElem = this->sdf->GetElement("inertia"); 
  this->SetInertiaMatrix( 
      inertiaElem->GetValueDouble("ixx"),
      inertiaElem->GetValueDouble("iyy"),
      inertiaElem->GetValueDouble("izz"),
      inertiaElem->GetValueDouble("ixy"),
      inertiaElem->GetValueDouble("ixz"),
      inertiaElem->GetValueDouble("iyz"));
      
  this->SetMass( this->sdf->GetValueDouble("mass") );
}

double Inertial::GetLinearDamping()
{
  double value = 0;
  if (this->sdf->HasElement("damping"))
  {
    sdf::ElementPtr dampingElem = this->sdf->GetElement("damping");
    value = dampingElem->GetValueDouble("linear");
  }

  return value;
}

double Inertial::GetAngularDamping()
{
  double value = 0;
  if (this->sdf->HasElement("damping"))
  {
    sdf::ElementPtr dampingElem = this->sdf->GetElement("damping");
    value = dampingElem->GetValueDouble("angular");
  }

  return value;
}

////////////////////////////////////////////////////////////////////////////////
/// Reset all the mass properties
void Inertial::Reset()
{
  this->mass = 0;
  this->cog.Set(0,0,0);
  this->principals.Set(0,0,0);
  this->products.Set(0,0,0);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the mass
void Inertial::SetMass(double m)
{
  this->mass = m;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the mass value
double Inertial::GetMass() const
{
  return this->mass;
}

////////////////////////////////////////////////////////////////////////////////
// Set the center of gravity
void Inertial::SetCoG(double cx, double cy, double cz)
{
  this->cog.Set(cx, cy, cz);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the center of gravity
void Inertial::SetCoG(const math::Vector3 &c)
{
  this->cog = c;
}

////////////////////////////////////////////////////////////////////////////////
// Get the center of gravity
math::Vector3 Inertial::GetCoG() const
{
  return this->cog;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the mass matrix
void Inertial::SetInertiaMatrix(double ixx, double iyy, double izz,
                                double ixy, double ixz, double iyz)
{
  this->principals.Set( ixx, iyy, izz );
  this->products.Set( ixy, ixz, iyz );
}


////////////////////////////////////////////////////////////////////////////////
/// Get the prinicpal moments of inertia (Ixx, Iyy, Izz)
math::Vector3 Inertial::GetPrincipalMoments() const
{
  return this->principals;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the products of inertia (Ixy, Ixy, Iyz)
math::Vector3 Inertial::GetProductsofInertia() const
{
  return this->products;
}

////////////////////////////////////////////////////////////////////////////////
/// Rotate this mass
void Inertial::Rotate(const math::Quaternion &rot)
{
  this->cog = rot.RotateVector(this->cog);
}

////////////////////////////////////////////////////////////////////////////////
/// Equal operator
void Inertial::operator=(const Inertial &_inertial)
{
  this->mass = _inertial.mass;
  this->cog = _inertial.cog;
  this->principals = _inertial.principals;
  this->products = _inertial.products;
}

Inertial Inertial::operator+(const Inertial &_inertial ) const
{
  Inertial result;
  result.mass = this->mass + _inertial.mass;

  result.cog = (this->cog*this->mass + _inertial.cog * _inertial.mass) / result.mass;

  result.principals = this->principals + _inertial.principals;
  result.products = this->products + _inertial.products;
  return result;
}

const Inertial &Inertial::operator+=(const Inertial &_inertial )
{
  *this = *this + _inertial;
  return *this;
}

