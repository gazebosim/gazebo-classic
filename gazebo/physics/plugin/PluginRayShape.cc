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
/* Desc: A ray
 * Author: Nate Koenig
 * Date: 14 Oct 2009
 */

#include "gazebo/common/Assert.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/plugin/PluginPhysics.hh"
#include "gazebo/physics/plugin/PluginTypes.hh"
#include "gazebo/physics/plugin/PluginCollision.hh"
#include "gazebo/physics/plugin/PluginRayShape.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
PluginRayShape::PluginRayShape(PhysicsEnginePtr _physicsEngine)
  : RayShape(_physicsEngine)
{
  this->SetName("Plugin Ray Shape");

  this->collisionParent.reset();
}


//////////////////////////////////////////////////
PluginRayShape::PluginRayShape(CollisionPtr _parent)
    : RayShape(_parent)
{
  GZ_ASSERT(_parent, "Parent collision shape is NULL");
  this->SetName("Plugin Ray Shape");

  PluginCollisionPtr collision =
    boost::static_pointer_cast<PluginCollision>(this->collisionParent);
}

//////////////////////////////////////////////////
PluginRayShape::~PluginRayShape()
{
  dGeomDestroy(this->geomId);
}

//////////////////////////////////////////////////
void PluginRayShape::Update()
{
  math::Vector3 dir;

  if (this->collisionParent)
  {
    PluginCollisionPtr collision =
      boost::static_pointer_cast<PluginCollision>(this->collisionParent);

    this->globalStartPos =
      this->collisionParent->GetLink()->GetWorldPose().CoordPositionAdd(
          this->relativeStartPos);

    this->globalEndPos =
      this->collisionParent->GetLink()->GetWorldPose().CoordPositionAdd(
          this->relativeEndPos);
  }

  dir = this->globalEndPos - this->globalStartPos;
  dir.Normalize();
}
