/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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

#include "gazebo/physics/dart/DARTMultiRayShapePrivate.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTMultiRayShape::DARTMultiRayShape(CollisionPtr _parent)
  : MultiRayShape(_parent),
    dataPtr(new DARTMultiRayShapePrivate())
{
  this->SetName("DART_multiray_shape");
}

//////////////////////////////////////////////////
DARTMultiRayShape::~DARTMultiRayShape()
{
  delete this->dataPtr;
}

//////////////////////////////////////////////////
void DARTMultiRayShape::UpdateRays()
{
  std::vector<RayShapePtr>::iterator iter;
  for (iter = this->rays.begin(); iter != this->rays.end(); ++iter)
  {
    (*iter)->Update();
  }
}

//////////////////////////////////////////////////
void DARTMultiRayShape::AddRay(const math::Vector3& _start,
                               const math::Vector3& _end)
{
  MultiRayShape::AddRay(_start, _end);

  DARTRayShapePtr ray(new DARTRayShape(this->collisionParent));
  ray->SetPoints(_start, _end);

  this->rays.push_back(ray);
}
