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
/* Desc: PluginCollision class
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 */

#include <sstream>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/math/Box.hh"

#include "gazebo/physics/PhysicsPlugin.h"
#include "gazebo/physics/plugin/PluginSurfaceParams.hh"
#include "gazebo/physics/plugin/PluginPhysics.hh"
#include "gazebo/physics/plugin/PluginLink.hh"
#include "gazebo/physics/plugin/PluginCollision.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
PluginCollision::PluginCollision(LinkPtr _link)
: Collision(_link)
{
  this->SetName("Plugin_Collision");
  this->onPoseChangeFunc = &PluginCollision::OnPoseChangeNull;

  this->surface.reset(new PluginSurfaceParams());
}

//////////////////////////////////////////////////
PluginCollision::~PluginCollision()
{
}

//////////////////////////////////////////////////
void PluginCollision::Load(sdf::ElementPtr _sdf)
{
  Collision::Load(_sdf);

  if (this->IsStatic())
  {
    this->SetCategoryBits(GZ_FIXED_COLLIDE);
    this->SetCollideBits(~GZ_FIXED_COLLIDE);
  }

  // Force max correcting velocity to zero for certain collision entities
  if (this->IsStatic() || this->shape->HasType(Base::HEIGHTMAP_SHAPE) ||
      this->shape->HasType(Base::MAP_SHAPE))
  {
    this->GetPluginSurface()->maxVel = 0.0;
  }
}

//////////////////////////////////////////////////
void PluginCollision::Fini()
{
  Collision::Fini();
}

//////////////////////////////////////////////////
void PluginCollision::OnPoseChange()
{
  // Update all the models
  (*this.*onPoseChangeFunc)();

  this->OnPoseChangeGlobal();
  this->OnPoseChangeRelative();
}

//////////////////////////////////////////////////
void PluginCollision::SetCategoryBits(unsigned int _bits)
{
}

//////////////////////////////////////////////////
void PluginCollision::SetCollideBits(unsigned int _bits)
{
}

//////////////////////////////////////////////////
math::Box PluginCollision::GetBoundingBox() const
{
  math::Box box;
  return box;
}

/////////////////////////////////////////////////
PluginSurfaceParamsPtr PluginCollision::GetPluginSurface() const
{
  return boost::dynamic_pointer_cast<PluginSurfaceParams>(this->surface);
}

/////////////////////////////////////////////////
void PluginCollision::OnPoseChangeGlobal()
{
}

/////////////////////////////////////////////////
void PluginCollision::OnPoseChangeRelative()
{
}

/////////////////////////////////////////////////
void PluginCollision::OnPoseChangeNull()
{
}
