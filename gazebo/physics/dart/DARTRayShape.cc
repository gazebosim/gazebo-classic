/*
 * Copyright 2015 Open Source Robotics Foundation
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

#include "gazebo/physics/World.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/dart/DARTPhysics.hh"
#include "gazebo/physics/dart/DARTTypes.hh"
#include "gazebo/physics/dart/DARTLink.hh"
#include "gazebo/physics/dart/DARTCollision.hh"
#include "gazebo/physics/dart/DARTRayShape.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTRayShape::DARTRayShape(PhysicsEnginePtr _physicsEngine)
  : RayShape(_physicsEngine)
{
  this->SetName("DART_ray_shape");
  this->physicsEngine =
    boost::static_pointer_cast<DARTPhysics>(_physicsEngine);
}

//////////////////////////////////////////////////
DARTRayShape::DARTRayShape(CollisionPtr _parent)
    : RayShape(_parent)
{
  this->SetName("DART_ray_shape");
  this->physicsEngine = boost::static_pointer_cast<DARTPhysics>(
      this->collisionParent->GetWorld()->GetPhysicsEngine());
}

//////////////////////////////////////////////////
DARTRayShape::~DARTRayShape()
{
}

//////////////////////////////////////////////////
void DARTRayShape::Update()
{
  // Not implemented yet, please see issue #911
}

//////////////////////////////////////////////////
void DARTRayShape::GetIntersection(double &_dist, std::string &_entity)
{
  _dist = 0;
  _entity = "";

  if (this->physicsEngine)
  {
  }

  // Not implemented yet, please see issue #911
}

//////////////////////////////////////////////////
void DARTRayShape::SetPoints(const math::Vector3& _posStart,
                             const math::Vector3& _posEnd)
{
  RayShape::SetPoints(_posStart, _posEnd);

  // Not implemented yet, please see issue #911
}
