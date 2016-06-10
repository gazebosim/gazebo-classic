/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include "gazebo/physics/bullet/BulletMesh.hh"
#include "gazebo/physics/bullet/BulletTypes.hh"
#include "gazebo/physics/bullet/BulletCollision.hh"
#include "gazebo/physics/bullet/BulletPhysics.hh"
#include "gazebo/physics/bullet/BulletMeshShape.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletMeshShape::BulletMeshShape(CollisionPtr _parent)
  : MeshShape(_parent)
{
  this->bulletMesh = new BulletMesh();
}

//////////////////////////////////////////////////
BulletMeshShape::~BulletMeshShape()
{
  delete this->bulletMesh;
}

//////////////////////////////////////////////////
void BulletMeshShape::Load(sdf::ElementPtr _sdf)
{
  MeshShape::Load(_sdf);
}

//////////////////////////////////////////////////
void BulletMeshShape::Init()
{
  MeshShape::Init();

  BulletCollisionPtr bParent =
    boost::static_pointer_cast<BulletCollision>(this->collisionParent);

  if (this->submesh)
  {
    this->bulletMesh->Init(this->submesh, bParent,
        this->sdf->Get<math::Vector3>("scale"));
  }
  else
  {
    this->bulletMesh->Init(this->mesh, bParent,
        this->sdf->Get<math::Vector3>("scale"));
  }
}
