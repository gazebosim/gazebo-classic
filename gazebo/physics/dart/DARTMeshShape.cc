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

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Mesh.hh"

#include "gazebo/physics/dart/DARTMesh.hh"
#include "gazebo/physics/dart/DARTCollision.hh"
#include "gazebo/physics/dart/DARTMeshShape.hh"
#include "gazebo/physics/dart/DARTPhysics.hh"

#include "gazebo/physics/dart/DARTMeshShapePrivate.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTMeshShape::DARTMeshShape(DARTCollisionPtr _parent)
  : MeshShape(_parent),
    dataPtr(new DARTMeshShapePrivate())
{
}

//////////////////////////////////////////////////
DARTMeshShape::~DARTMeshShape()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
void DARTMeshShape::Update()
{
  MeshShape::Update();
}

//////////////////////////////////////////////////
void DARTMeshShape::Load(sdf::ElementPtr _sdf)
{
  MeshShape::Load(_sdf);
}

//////////////////////////////////////////////////
void DARTMeshShape::Init()
{
  MeshShape::Init();


  if (this->submesh)
  {
    this->dataPtr->dartMesh->Init(this->submesh,
        boost::dynamic_pointer_cast<DARTCollision>(this->collisionParent),
        this->sdf->Get<ignition::math::Vector3d>("scale"));
  }
  else
  {
    if (!this->mesh)
    {
      gzerr << "No DART mesh specified\n";
      return;
    }
    this->dataPtr->dartMesh->Init(this->mesh,
        boost::dynamic_pointer_cast<DARTCollision>(this->collisionParent),
        this->sdf->Get<ignition::math::Vector3d>("scale"));
  }

  BasePtr _parent = GetParent();
  GZ_ASSERT(boost::dynamic_pointer_cast<DARTCollision>(_parent),
            "Parent must be a DARTCollisionPtr");
  DARTCollisionPtr _collisionParent =
    boost::static_pointer_cast<DARTCollision>(_parent);

  GZ_ASSERT(this->dataPtr->dartMesh->ShapeNode(), "Must have a shape node");
  _collisionParent->SetDARTCollisionShapeNode
     (this->dataPtr->dartMesh->ShapeNode());
}
