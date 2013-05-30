/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include "gazebo/common/Exception.hh"

#include "gazebo/physics/World.hh"
#include "gazebo/physics/dart/DARTTypes.hh"
#include "gazebo/physics/dart/DARTLink.hh"
#include "gazebo/physics/dart/DARTCollision.hh"
#include "gazebo/physics/dart/DARTPhysics.hh"
#include "gazebo/physics/dart/DARTRayShape.hh"
#include "gazebo/physics/dart/DARTMultiRayShape.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
DARTMultiRayShape::DARTMultiRayShape(CollisionPtr _parent)
  : MultiRayShape(_parent)
{
}

//////////////////////////////////////////////////
DARTMultiRayShape::~DARTMultiRayShape()
{
}

//////////////////////////////////////////////////
void DARTMultiRayShape::UpdateRays()
{
  gzwarn << "Not implemented!\n";
}
