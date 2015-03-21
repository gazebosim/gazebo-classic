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

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Mesh.hh"

#include "gazebo/physics/dart/DARTMesh.hh"
#include "gazebo/physics/dart/DARTCollision.hh"
#include "gazebo/physics/dart/DARTMeshShape.hh"
#include "gazebo/physics/dart/DARTPhysics.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTMeshShape::DARTMeshShape(CollisionPtr _parent) : MeshShape(_parent)
{
  this->dartMesh = new DARTMesh();
}

//////////////////////////////////////////////////
DARTMeshShape::~DARTMeshShape()
{
  delete this->dartMesh;
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
    this->dartMesh->Init(this->submesh,
        boost::dynamic_pointer_cast<DARTCollision>(this->collisionParent),
        this->sdf->Get<math::Vector3>("scale"));
  }
  else
  {
    this->dartMesh->Init(this->mesh,
        boost::dynamic_pointer_cast<DARTCollision>(this->collisionParent),
        this->sdf->Get<math::Vector3>("scale"));
  }
}
