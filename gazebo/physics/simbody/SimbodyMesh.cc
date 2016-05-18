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

#include "gazebo/physics/simbody/SimbodyTypes.hh"
#include "gazebo/physics/simbody/SimbodyCollision.hh"
#include "gazebo/physics/simbody/SimbodyPhysics.hh"
#include "gazebo/physics/simbody/SimbodyMesh.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodyMesh::SimbodyMesh()
{
}

//////////////////////////////////////////////////
SimbodyMesh::~SimbodyMesh()
{
}

//////////////////////////////////////////////////
void SimbodyMesh::Init(const common::SubMesh * /*_subMesh*/,
                      SimbodyCollisionPtr /*_collision*/,
                      const math::Vector3 & /*_scale*/)
{
  gzerr << "SimbodyMesh is not supported\n";
}

//////////////////////////////////////////////////
void SimbodyMesh::Init(const common::Mesh * /*_mesh*/,
                       SimbodyCollisionPtr /*_collision*/,
                       const math::Vector3 &/*_scale*/)
{
  gzerr << "SimbodyMesh is not supported\n";
}

//////////////////////////////////////////////////
void SimbodyMesh::CreateMesh(float * /*_vertices*/, int * /*_indices*/,
    unsigned int /*_numVertices*/, unsigned int /*_numIndices*/,
    SimbodyCollisionPtr /*_collision*/,
    const math::Vector3 & /*_scale*/)
{
}
