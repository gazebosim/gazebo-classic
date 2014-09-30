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
/* Desc: Plugin Heightmap shape
 * Author: Nate Koenig
 * Date: 12 Nov 2009
 */

#include "gazebo/common/Exception.hh"
#include "gazebo/physics/PhysicsPlugin.h"
#include "gazebo/physics/plugin/PluginCollision.hh"
#include "gazebo/physics/plugin/PluginHeightmapShape.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
PluginHeightmapShape::PluginHeightmapShape(CollisionPtr _parent)
    : HeightmapShape(_parent)
{
  this->flipY = false;
}

//////////////////////////////////////////////////
PluginHeightmapShape::~PluginHeightmapShape()
{
}

//////////////////////////////////////////////////
void PluginHeightmapShape::Init()
{
  HeightmapShape::Init();

  PluginCollisionPtr oParent =
    boost::static_pointer_cast<PluginCollision>(this->collisionParent);

  oParent->SetStatic(true);

  // Rotate so Z is up, not Y (which is the default orientation)
  math::Quaternion quat;
  math::Pose pose = oParent->GetWorldPose();

  // TODO: FIXME:  double check this, if Y is up,
  // rotating by roll of 90 deg will put Z-down.
  quat.SetFromEuler(math::Vector3(GZ_DTOR(90), 0, 0));

  pose.rot = pose.rot * quat;
  // this->body->SetPose(pose);
}
