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
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/World.hh"
#include "gazebo/physics/plugin/PluginTypes.hh"
#include "gazebo/physics/plugin/PluginLink.hh"
#include "gazebo/physics/plugin/PluginCollision.hh"
#include "gazebo/physics/plugin/PluginPhysics.hh"
#include "gazebo/physics/plugin/PluginRayShape.hh"
#include "gazebo/physics/plugin/PluginMultiRayShape.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
PluginMultiRayShape::PluginMultiRayShape(CollisionPtr _parent)
  : MultiRayShape(_parent)
{
  this->SetName("Plugin Multiray Shape");
}

//////////////////////////////////////////////////
PluginMultiRayShape::~PluginMultiRayShape()
{
}

//////////////////////////////////////////////////
void PluginMultiRayShape::UpdateRays()
{
  PluginPhysicsPtr plugin = boost::dynamic_pointer_cast<PluginPhysics>(
      this->GetWorld()->GetPhysicsEngine());

  if (plugin == NULL)
    gzthrow("Invalid physics engine. Must use Plugin.");

  // Do we need to lock the physics engine here? YES!
  // especially when spawning models with sensors
  {
    boost::recursive_mutex::scoped_lock lock(*plugin->GetPhysicsUpdateMutex());

    // Do collision detection
  }
}

//////////////////////////////////////////////////
void PluginMultiRayShape::AddRay(const math::Vector3 &_start,
    const math::Vector3 &_end)
{
  MultiRayShape::AddRay(_start, _end);
}
