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

#include "gazebo/common/Exception.hh"

#include "gazebo/physics/World.hh"
#include "gazebo/physics/bullet/BulletTypes.hh"
#include "gazebo/physics/bullet/BulletLink.hh"
#include "gazebo/physics/bullet/BulletCollision.hh"
#include "gazebo/physics/bullet/BulletPhysics.hh"
#include "gazebo/physics/bullet/BulletRayShape.hh"
#include "gazebo/physics/bullet/BulletMultiRayShape.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletMultiRayShape::BulletMultiRayShape(CollisionPtr _parent)
: MultiRayShape(_parent)
{
  this->SetName("Bullet Multiray Shape");
  this->physicsEngine = boost::static_pointer_cast<BulletPhysics>(
      this->collisionParent->GetWorld()->GetPhysicsEngine());
}

//////////////////////////////////////////////////
BulletMultiRayShape::~BulletMultiRayShape()
{
}

//////////////////////////////////////////////////
void BulletMultiRayShape::UpdateRays()
{
  std::vector<RayShapePtr>::iterator iter;
  for (iter = this->rays.begin(); iter != this->rays.end(); ++iter)
  {
    (*iter)->Update();
  }
}

//////////////////////////////////////////////////
void BulletMultiRayShape::AddRay(const math::Vector3 &_start,
    const math::Vector3 &_end)
{
  MultiRayShape::AddRay(_start, _end);

  BulletRayShapePtr ray(new BulletRayShape(this->collisionParent));
  ray->SetPoints(_start, _end);

  this->rays.push_back(ray);
}
