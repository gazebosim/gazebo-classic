/*
 * Copyright 2014 Open Source Robotics Foundation
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
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/dart/DARTCollision.hh"
#include "gazebo/physics/dart/DARTPhysics.hh"
#include "gazebo/physics/dart/DARTMeshShape.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
DARTMeshShape::DARTMeshShape(CollisionPtr _parent) : MeshShape(_parent)
{
}

//////////////////////////////////////////////////
DARTMeshShape::~DARTMeshShape()
{
}

//////////////////////////////////////////////////
void DARTMeshShape::Update()
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTMeshShape::Load(sdf::ElementPtr _sdf)
{
  MeshShape::Load(_sdf);
}

//////////////////////////////////////////////////
void DARTMeshShape::Init()
{
  gzwarn << "Not implemented!\n";
}
