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
#include "gazebo/common/Mesh.hh"
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/plugin/PluginMesh.hh"
#include "gazebo/physics/plugin/PluginCollision.hh"
#include "gazebo/physics/plugin/PluginPhysics.hh"
#include "gazebo/physics/plugin/PluginMeshShape.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
PluginMeshShape::PluginMeshShape(CollisionPtr _parent) : MeshShape(_parent)
{
  this->odeMesh = new PluginMesh();
}

//////////////////////////////////////////////////
PluginMeshShape::~PluginMeshShape()
{
  delete this->odeMesh;
}

//////////////////////////////////////////////////
void PluginMeshShape::Update()
{
  this->odeMesh->Update();
}

//////////////////////////////////////////////////
void PluginMeshShape::Load(sdf::ElementPtr _sdf)
{
  MeshShape::Load(_sdf);
}

//////////////////////////////////////////////////
void PluginMeshShape::Init()
{
  MeshShape::Init();
  if (!this->mesh)
    return;

  if (this->submesh)
  {
    this->odeMesh->Init(this->submesh,
        boost::static_pointer_cast<PluginCollision>(this->collisionParent),
        this->sdf->Get<math::Vector3>("scale"));
  }
  else
  {
    this->odeMesh->Init(this->mesh,
        boost::static_pointer_cast<PluginCollision>(this->collisionParent),
        this->sdf->Get<math::Vector3>("scale"));
  }
}
