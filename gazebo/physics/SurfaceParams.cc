/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include "gazebo/math/Helpers.hh"
#include "gazebo/physics/SurfaceParams.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
FrictionPyramid::FrictionPyramid()
{
  this->mu[0] = 1.0;
  this->mu[1] = 1.0;
}

//////////////////////////////////////////////////
FrictionPyramid::~FrictionPyramid()
{
}

//////////////////////////////////////////////////
double FrictionPyramid::GetMuPrimary()
{
  return this->GetMu(0);
}

//////////////////////////////////////////////////
double FrictionPyramid::GetMuSecondary()
{
  return this->GetMu(1);
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
double FrictionPyramid::GetMu(unsigned int _index)
{
  GZ_ASSERT(_index < 2, "Invalid _index to GetMu");
  return this->mu[_index];
}

//////////////////////////////////////////////////
void FrictionPyramid::SetMu(unsigned int _index, double _mu)
{
  GZ_ASSERT(_index < 2, "Invalid _index to SetMu");
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
    collideBitmask(1)
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
