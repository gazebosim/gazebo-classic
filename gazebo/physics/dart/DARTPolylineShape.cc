/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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

#include "gazebo/common/Mesh.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/physics/dart/DARTMesh.hh"
#include "gazebo/physics/dart/DARTCollision.hh"
#include "gazebo/physics/dart/DARTPhysics.hh"
#include "gazebo/physics/dart/DARTPolylineShape.hh"

#include "gazebo/physics/dart/DARTPolylineShapePrivate.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTPolylineShape::DARTPolylineShape(CollisionPtr _parent)
  : PolylineShape(_parent),
    dataPtr(new DARTPolylineShapePrivate())
{
}

//////////////////////////////////////////////////
DARTPolylineShape::~DARTPolylineShape()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
void DARTPolylineShape::Load(sdf::ElementPtr _sdf)
{
  PolylineShape::Load(_sdf);
}

//////////////////////////////////////////////////
void DARTPolylineShape::Init()
{
  PolylineShape::Init();
  if (!this->mesh)
  {
    gzerr << "Unable to create polyline in DART. Mesh pointer is null.\n";
    return;
  }

  this->dataPtr->dartMesh->Init(this->mesh,
      boost::static_pointer_cast<DARTCollision>(this->collisionParent),
      math::Vector3(1, 1, 1));
}
