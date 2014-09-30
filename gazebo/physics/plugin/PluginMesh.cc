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
#include "gazebo/common/Mesh.hh"
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/plugin/PluginCollision.hh"
#include "gazebo/physics/plugin/PluginPhysics.hh"
#include "gazebo/physics/plugin/PluginMesh.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
PluginMesh::PluginMesh()
{
  this->vertices = NULL;
  this->indices = NULL;
}

//////////////////////////////////////////////////
PluginMesh::~PluginMesh()
{
  delete [] this->vertices;
  delete [] this->indices;
}

//////////////////////////////////////////////////
void PluginMesh::Update()
{
}

//////////////////////////////////////////////////
void PluginMesh::Init(const common::SubMesh *_subMesh, PluginCollisionPtr _collision,
    const math::Vector3 &_scale)
{
}

//////////////////////////////////////////////////
void PluginMesh::Init(const common::Mesh *_mesh, PluginCollisionPtr _collision,
    const math::Vector3 &_scale)
{
}
