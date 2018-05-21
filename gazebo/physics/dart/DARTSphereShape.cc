/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#include "gazebo/common/Console.hh"
#include "gazebo/physics/dart/DARTPhysics.hh"
#include "gazebo/physics/dart/DARTCollision.hh"
#include "gazebo/physics/dart/DARTSphereShape.hh"
#include "gazebo/util/system.hh"

#include "gazebo/physics/dart/DARTSphereShapePrivate.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTSphereShape::DARTSphereShape(DARTCollisionPtr _parent)
  : SphereShape(_parent),
    dataPtr(new DARTSphereShapePrivate())
{
}

//////////////////////////////////////////////////
DARTSphereShape::~DARTSphereShape()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
void DARTSphereShape::Init()
{
  BasePtr _parent = GetParent();

  GZ_ASSERT(boost::dynamic_pointer_cast<DARTCollision>(_parent),
            "Parent must be a DARTCollisionPtr");
  DARTCollisionPtr _collisionParent =
    boost::static_pointer_cast<DARTCollision>(_parent);

  dart::dynamics::BodyNodePtr bodyNode = _collisionParent->DARTBodyNode();

  if (!bodyNode.get()) gzerr << "BodyNode is NULL in init!\n";
  GZ_ASSERT(bodyNode, "BodyNode is NULL in init!");

  this->dataPtr->CreateShape(bodyNode);
  _collisionParent->SetDARTCollisionShapeNode(
                       this->dataPtr->ShapeNode(), false);

  SphereShape::Init();
}


//////////////////////////////////////////////////
void DARTSphereShape::SetRadius(double _radius)
{
  if (_radius < 0)
  {
    gzerr << "Sphere shape does not support negative radius.\n";
    return;
  }
  if (ignition::math::equal(_radius, 0.0))
  {
    // Warn user, but still create shape with very small value
    // otherwise later resize operations using setLocalScaling
    // will not be possible
    gzwarn << "Setting sphere shape's radius to zero is not supported "
           << "in DART, using 1e-4.\n";
    _radius = 1e-4;
  }

  SphereShape::SetRadius(_radius);

  this->dataPtr->Shape()->setRadius(_radius);
}

