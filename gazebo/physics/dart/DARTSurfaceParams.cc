/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "gazebo/common/Console.hh"
#include "gazebo/physics/dart/DARTSurfaceParams.hh"

#include "gazebo/physics/dart/DARTSurfaceParamsPrivate.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTSurfaceParams::DARTSurfaceParams()
  : SurfaceParams(),
    dataPtr(new DARTSurfaceParamsPrivate())
{
}

//////////////////////////////////////////////////
DARTSurfaceParams::~DARTSurfaceParams()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
void DARTSurfaceParams::Load(sdf::ElementPtr _sdf)
{
  // Load parent class
  SurfaceParams::Load(_sdf);

  if (!_sdf)
    gzerr << "Surface _sdf is null" << std::endl;
  else
  {
    sdf::ElementPtr frictionElem = _sdf->GetElement("friction");
    if (!frictionElem)
      gzerr << "Surface friction sdf member is null" << std::endl;
    {
      // Note this should not be looking in the "ode" block
      // Update this when sdformat has bullet friction parameters
      // See sdformat issue #31
      // https://github.com/osrf/sdformat/issue/31
      sdf::ElementPtr frictionOdeElem = frictionElem->GetElement("ode");
      if (!frictionOdeElem)
        gzerr << "Surface friction ode sdf member is null" << std::endl;
      else
      {
        this->dataPtr->frictionPyramid->SetMuPrimary(
          frictionOdeElem->Get<double>("mu"));
        this->dataPtr->frictionPyramid->SetMuSecondary(
          frictionOdeElem->Get<double>("mu2"));
        this->dataPtr->frictionPyramid->direction1 =
          frictionOdeElem->Get<ignition::math::Vector3d>("fdir1");
      }
    }
  }
}

/////////////////////////////////////////////////
void DARTSurfaceParams::FillMsg(msgs::Surface &_msg)
{
  SurfaceParams::FillMsg(_msg);

  _msg.mutable_friction()->set_mu(
        this->dataPtr->frictionPyramid->MuPrimary());
  _msg.mutable_friction()->set_mu2(
        this->dataPtr->frictionPyramid->MuSecondary());
}

/////////////////////////////////////////////////
void DARTSurfaceParams::ProcessMsg(const msgs::Surface &_msg)
{
  SurfaceParams::ProcessMsg(_msg);

  if (_msg.has_friction())
  {
    if (_msg.friction().has_mu())
      this->dataPtr->frictionPyramid->SetMuPrimary(_msg.friction().mu());
    if (_msg.friction().has_mu2())
      this->dataPtr->frictionPyramid->SetMuSecondary(_msg.friction().mu2());
  }
}

/////////////////////////////////////////////////
FrictionPyramidPtr DARTSurfaceParams::FrictionPyramid() const
{
  return this->dataPtr->frictionPyramid;
}
