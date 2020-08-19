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
#include "gazebo/common/Exception.hh"
#include "gazebo/physics/dart/DARTCollision.hh"
#include "gazebo/physics/dart/DARTHeightmapShape.hh"

#include "gazebo/physics/dart/DARTHeightmapShapePrivate.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTHeightmapShape::DARTHeightmapShape(DARTCollisionPtr _parent)
  : HeightmapShape(_parent),
    dataPtr(new DARTHeightmapShapePrivate<HeightmapShape::HeightType>())
{
  this->flipY = false;
}

//////////////////////////////////////////////////
DARTHeightmapShape::~DARTHeightmapShape()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
void DARTHeightmapShape::Init()
{
  BasePtr _parent = GetParent();
  GZ_ASSERT(boost::dynamic_pointer_cast<DARTCollision>(_parent),
            "Parent must be a DARTCollisionPtr");
  DARTCollisionPtr _collisionParent =
    boost::static_pointer_cast<DARTCollision>(_parent);

  dart::dynamics::BodyNodePtr bodyNode = _collisionParent->DARTBodyNode();

  if (!bodyNode) gzerr << "BodyNode is NULL in Init!\n";
  GZ_ASSERT(bodyNode, "BodyNode is NULL Init!");

  this->dataPtr->CreateShape(bodyNode);
  _collisionParent->SetDARTCollisionShapeNode(
                      this->dataPtr->ShapeNode(), false);

  // superclasses' Init method initializes the heightmap
  // data (this->heights etc.)
  HeightmapShape::Init();

  GZ_ASSERT(this->dataPtr->Shape(), "Shape is NULL");
  this->dataPtr->Shape()->setHeightField(this->vertSize, this->vertSize,
                                         this->heights);
  this->dataPtr->Shape()->setScale(Vector3(this->scale.X(),
                                           this->scale.Y(), 1));
}

//////////////////////////////////////////////////
void DARTHeightmapShape::SetScale(const ignition::math::Vector3d &_scale)
{
  GZ_ASSERT(this->dataPtr->Shape(), "Shape is NULL");
  this->dataPtr->Shape()->setScale(Vector3(_scale.X(), _scale.Y(), 1));
  HeightmapShape::SetScale(_scale);
}
