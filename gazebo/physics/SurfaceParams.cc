/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/physics/SurfaceParams.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
FrictionPyramid::FrictionPyramid()
  : patchRadius(0.0)
  , surfaceRadius(IGN_DBL_MAX)
  , usePatchRadius(1)
  , poissonsRatio(0.3)
  , elasticModulus(0)
{
  this->mu[0] = 1.0;
  this->mu[1] = 1.0;
  this->mu[2] = 1.0;
}

//////////////////////////////////////////////////
FrictionPyramid::~FrictionPyramid()
{
}

//////////////////////////////////////////////////
double FrictionPyramid::MuPrimary() const
{
  return this->Mu(0);
}

//////////////////////////////////////////////////
double FrictionPyramid::MuSecondary() const
{
  return this->Mu(1);
}

//////////////////////////////////////////////////
double FrictionPyramid::MuTorsion() const
{
  return this->Mu(2);
}

//////////////////////////////////////////////////
double FrictionPyramid::PatchRadius() const
{
  return this->patchRadius;
}

//////////////////////////////////////////////////
double FrictionPyramid::SurfaceRadius() const
{
  return this->surfaceRadius;
}

//////////////////////////////////////////////////
bool FrictionPyramid::UsePatchRadius() const
{
  return this->usePatchRadius;
}

//////////////////////////////////////////////////
void FrictionPyramid::SetMuPrimary(double _mu)
{
  this->SetMu(0, _mu);
}

//////////////////////////////////////////////////
void FrictionPyramid::SetMuSecondary(double _mu)
{
  this->SetMu(1, _mu);
}

//////////////////////////////////////////////////
void FrictionPyramid::SetMuTorsion(const double _mu)
{
  this->SetMu(2, _mu);
}

//////////////////////////////////////////////////
void FrictionPyramid::SetPatchRadius(const double _radius)
{
  this->patchRadius = _radius;
}

//////////////////////////////////////////////////
void FrictionPyramid::SetSurfaceRadius(const double _radius)
{
  this->surfaceRadius = _radius;
}

//////////////////////////////////////////////////
void FrictionPyramid::SetUsePatchRadius(const bool _use)
{
  this->usePatchRadius = _use;
}

//////////////////////////////////////////////////
double FrictionPyramid::PoissonsRatio() const
{
  return this->poissonsRatio;
}

//////////////////////////////////////////////////
void FrictionPyramid::SetPoissonsRatio(const double _ratio)
{
  if (_ratio < -1.0)
  {
    this->poissonsRatio = -1.00;
    gzwarn << "Poisson's ratio out of bounds [" << _ratio
      << "] default to [" << this->poissonsRatio << "]\n";
  }
  else if (_ratio > 0.5)
  {
    this->poissonsRatio = 0.5;
    gzwarn << "Poisson's ratio out of bounds [" << _ratio
      << "] default to [" << this->poissonsRatio << "]\n";
  }
  else
  {
    this->poissonsRatio = _ratio;
  }
}

//////////////////////////////////////////////////
double FrictionPyramid::ElasticModulus() const
{
  return this->elasticModulus;
}

//////////////////////////////////////////////////
void FrictionPyramid::SetElasticModulus(const double _modulus)
{
  if (_modulus < 0)
  {
    this->elasticModulus = 0;
  }
  else
  {
    this->elasticModulus = _modulus;
  }
}

//////////////////////////////////////////////////
double FrictionPyramid::Mu(const unsigned int _index) const
{
  GZ_ASSERT(_index < 3, "Invalid _index to Mu");
  return this->mu[_index];
}

//////////////////////////////////////////////////
void FrictionPyramid::SetMu(unsigned int _index, double _mu)
{
  GZ_ASSERT(_index < 3, "Invalid _index to SetMu");
  if (_mu < 0)
  {
    this->mu[_index] = GZ_FLT_MAX;
  }
  else
  {
    this->mu[_index] = _mu;
  }
}

//////////////////////////////////////////////////
SurfaceParams::SurfaceParams()
  : collideWithoutContact(false),
    collideWithoutContactBitmask(1),
    collideBitmask(65535)
{
}

//////////////////////////////////////////////////
SurfaceParams::~SurfaceParams()
{
}

//////////////////////////////////////////////////
void SurfaceParams::Load(sdf::ElementPtr _sdf)
{
  if (!_sdf)
    gzerr << "Surface _sdf is NULL" << std::endl;
  else
  {
    sdf::ElementPtr contactElem = _sdf->GetElement("contact");
    if (!contactElem)
      gzerr << "Surface contact sdf member is NULL" << std::endl;
    else
    {
      this->collideWithoutContact =
        contactElem->Get<bool>("collide_without_contact");
      this->collideWithoutContactBitmask =
          contactElem->Get<unsigned int>("collide_without_contact_bitmask");

      if (contactElem->HasElement("collide_bitmask"))
      {
        this->collideBitmask =
          contactElem->Get<unsigned int>("collide_bitmask");
      }
    }
  }
}

/////////////////////////////////////////////////
void SurfaceParams::FillMsg(msgs::Surface &_msg)
{
  _msg.set_collide_without_contact(this->collideWithoutContact);
  _msg.set_collide_without_contact_bitmask(this->collideWithoutContactBitmask);
  _msg.set_collide_bitmask(this->collideBitmask);
}

/////////////////////////////////////////////////
void SurfaceParams::ProcessMsg(const msgs::Surface &_msg)
{
  if (_msg.has_collide_without_contact())
    this->collideWithoutContact = _msg.collide_without_contact();
  if (_msg.has_collide_without_contact_bitmask())
    this->collideWithoutContactBitmask = _msg.collide_without_contact_bitmask();
  if (_msg.has_collide_bitmask())
    this->collideBitmask = _msg.collide_bitmask();
}

/////////////////////////////////////////////////
FrictionPyramidPtr SurfaceParams::FrictionPyramid() const
{
  return FrictionPyramidPtr();
}
