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

#include <float.h>
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/physics/bullet/BulletSurfaceParams.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletSurfaceParams::BulletSurfaceParams()
  : SurfaceParams()
  , frictionPyramid(new FrictionPyramid())
{
}

//////////////////////////////////////////////////
BulletSurfaceParams::~BulletSurfaceParams()
{
}

//////////////////////////////////////////////////
void BulletSurfaceParams::Load(sdf::ElementPtr _sdf)
{
  // Load parent class
  SurfaceParams::Load(_sdf);

  GZ_ASSERT(_sdf, "Surface _sdf is NULL");
  sdf::ElementPtr frictionElem = _sdf->GetElement("friction");
  GZ_ASSERT(frictionElem, "Surface friction sdf member is NULL");

  // Note this should not be looking in the "ode" block
  // Update this when sdformat has bullet friction parameters
  // See sdformat issue #31
  // https://bitbucket.org/osrf/sdformat/issue/31
  sdf::ElementPtr frictionOdeElem = frictionElem->GetElement("ode");
  GZ_ASSERT(frictionOdeElem , "Surface friction ode sdf member is NULL");

  this->frictionPyramid->SetMuPrimary(  frictionOdeElem->Get<double>("mu"));
  this->frictionPyramid->SetMuSecondary(frictionOdeElem->Get<double>("mu2"));
}

/////////////////////////////////////////////////
void BulletSurfaceParams::FillMsg(msgs::Surface &_msg)
{
  SurfaceParams::FillMsg(_msg);

  _msg.mutable_friction()->set_mu(this->frictionPyramid->GetMuPrimary());
  _msg.mutable_friction()->set_mu2(this->frictionPyramid->GetMuSecondary());
}

/////////////////////////////////////////////////
void BulletSurfaceParams::ProcessMsg(const msgs::Surface &_msg)
{
  SurfaceParams::ProcessMsg(_msg);

  if (_msg.has_friction())
  {
    if (_msg.friction().has_mu())
      this->frictionPyramid->SetMuPrimary(_msg.friction().mu());
    if (_msg.friction().has_mu2())
      this->frictionPyramid->SetMuSecondary(_msg.friction().mu2());
  }
}

/////////////////////////////////////////////////
FrictionPyramidPtr BulletSurfaceParams::GetFrictionPyramid() const
{
  return this->frictionPyramid;
}
