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
/* Desc: common::Parameters for contact joints
 * Author: Nate Koenig
 * Date: 30 July 2003
 */

#include <float.h>

#include "gazebo/common/Console.hh"
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
double FrictionPyramid::GetMu(unsigned int _index)
{
  if (_index >= 2)
  {
    gzerr << "Invalid index [" << _index << "]: should be "
          << "0 or 1 for FrictionPyramid::GetMu."
          << std::endl;
    return -1;
  }
  return this->mu[_index];
}

//////////////////////////////////////////////////
void FrictionPyramid::SetMu(unsigned int _index, double _mu)
{
  if (_index >= 2)
  {
    gzerr << "Invalid index [" << _index << "]: should be "
          << "0 or 1 for FrictionPyramid::SetMu."
          << std::endl;
  }
  this->mu[_index] = _mu;
  if (_mu < 0)
    this->mu[_index] = FLT_MAX;
}

//////////////////////////////////////////////////
SurfaceParams::SurfaceParams()
  : collideWithoutContact(false),
    collideWithoutContactBitmask(1)
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
    }
  }
}

/////////////////////////////////////////////////
void SurfaceParams::FillMsg(msgs::Surface &_msg)
{
  _msg.set_collide_without_contact(this->collideWithoutContact);
  _msg.set_collide_without_contact_bitmask(this->collideWithoutContactBitmask);
}


void SurfaceParams::ProcessMsg(const msgs::Surface &_msg)
{
  if (_msg.has_collide_without_contact())
    this->collideWithoutContact = _msg.collide_without_contact();
  if (_msg.has_collide_without_contact_bitmask())
    this->collideWithoutContactBitmask = _msg.collide_without_contact_bitmask();
}
