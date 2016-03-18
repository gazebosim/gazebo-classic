/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#include "gazebo/physics/ShapePrivate.hh"
#include "gazebo/physics/bullet/BulletPhysics.hh"
#include "gazebo/physics/bullet/BulletCollision.hh"
#include "gazebo/physics/bullet/BulletPlaneShape.hh"

using namespace gazebo;
using namespace physics;

/////////////////////////////////////////////////
BulletPlaneShape::BulletPlaneShape(CollisionPtr _parent)
: PlaneShape(_parent)
{
}

/////////////////////////////////////////////////
BulletPlaneShape::~BulletPlaneShape()
{
}

/////////////////////////////////////////////////
void BulletPlaneShape::CreatePlane()
{
  PlaneShape::CreatePlane();
  BulletCollisionPtr bParent;
  bParent = std::dynamic_pointer_cast<BulletCollision>(
      this->shapeDPtr->collisionParent);

  ignition::math::Vector3d n = this->Normal();
  btVector3 vec(n.X(), n.Y(), n.Z());

  bParent->SetCollisionShape(new btStaticPlaneShape(vec, 0.0),
      false);
}

