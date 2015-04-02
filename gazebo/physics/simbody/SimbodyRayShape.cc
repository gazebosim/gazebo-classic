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

#include "gazebo/physics/World.hh"
#include "gazebo/physics/simbody/SimbodyLink.hh"
#include "gazebo/physics/simbody/SimbodyPhysics.hh"
#include "gazebo/physics/simbody/SimbodyTypes.hh"
#include "gazebo/physics/simbody/SimbodyCollision.hh"
#include "gazebo/physics/simbody/SimbodyRayShape.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodyRayShape::SimbodyRayShape(PhysicsEnginePtr _physicsEngine)
  : RayShape(_physicsEngine)
{
  this->SetName("Simbody Ray Shape");

  this->physicsEngine =
    boost::static_pointer_cast<SimbodyPhysics>(_physicsEngine);
}

//////////////////////////////////////////////////
SimbodyRayShape::SimbodyRayShape(CollisionPtr _parent)
    : RayShape(_parent)
{
  this->SetName("Simbody Ray Shape");
  this->physicsEngine = boost::static_pointer_cast<SimbodyPhysics>(
      this->collisionParent->GetWorld()->GetPhysicsEngine());
}

//////////////////////////////////////////////////
SimbodyRayShape::~SimbodyRayShape()
{
}

//////////////////////////////////////////////////
void SimbodyRayShape::Update()
{
}

//////////////////////////////////////////////////
void SimbodyRayShape::GetIntersection(double &_dist, std::string &_entity)
{
  _dist = 0;
  _entity = "";

  if (this->physicsEngine)
  {
  }
}

//////////////////////////////////////////////////
void SimbodyRayShape::SetPoints(const math::Vector3 &_posStart,
                                   const math::Vector3 &_posEnd)
{
  this->globalStartPos = _posStart;
  this->globalEndPos = _posEnd;
}
