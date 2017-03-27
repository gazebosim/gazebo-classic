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
#include "gazebo/physics/dart/DARTPlaneShape.hh"
#include "gazebo/util/system.hh"

#include "gazebo/physics/dart/DARTPlaneShapePrivate.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTPlaneShape::DARTPlaneShape(DARTCollisionPtr _parent)
  : PlaneShape(_parent),
    dataPtr(new DARTPlaneShapePrivate())
{
}

//////////////////////////////////////////////////
DARTPlaneShape::~DARTPlaneShape()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
void DARTPlaneShape::Init()
{
  BasePtr _parent = GetParent();
  GZ_ASSERT(boost::dynamic_pointer_cast<DARTCollision>(_parent),
            "Parent must be a DARTCollisionPtr");
  DARTCollisionPtr _collisionParent =
    boost::static_pointer_cast<DARTCollision>(_parent);

  dart::dynamics::BodyNodePtr bodyNode = _collisionParent->DARTBodyNode();
  if (!bodyNode) gzerr << "BodyNode is NULL in init!\n";
  GZ_ASSERT(bodyNode, "BodyNode is NULL in init!");

  this->dataPtr->CreateShape(bodyNode);

  _collisionParent->SetDARTCollisionShapeNode(this->dataPtr->ShapeNode(),
                                              false);

  PlaneShape::Init();
}

//////////////////////////////////////////////////
void DARTPlaneShape::CreatePlane()
{
  PlaneShape::CreatePlane();
}

//////////////////////////////////////////////////
void DARTPlaneShape::SetAltitude(const ignition::math::Vector3d &_pos)
{
  PlaneShape::SetAltitude(_pos);
}

