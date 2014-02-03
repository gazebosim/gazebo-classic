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
/* Desc: Trimesh shape
 * Author: Nate Koenig
 * Date: 21 May 2009
 */

#include "gazebo/common/Mesh.hh"

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
}


//////////////////////////////////////////////////
BulletMeshShape::~BulletMeshShape()
{
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

  float *vertices = NULL;
  int *indices = NULL;

  btTriangleMesh *mTriMesh = new btTriangleMesh();

  unsigned int numVertices = this->mesh->GetVertexCount();
  unsigned int numIndices = this->mesh->GetIndexCount();

  // Get all the vertex and index data
  this->mesh->FillArrays(&vertices, &indices);

  // Scale the vertex data
  for (unsigned int j = 0;  j < numVertices; j++)
  {
    vertices[j*3+0] = vertices[j*3+0] *
      this->sdf->Get<math::Vector3>("scale").x;
    vertices[j*3+1] = vertices[j*3+1] *
      this->sdf->Get<math::Vector3>("scale").y;
    vertices[j*3+2] = vertices[j*3+2] *
      this->sdf->Get<math::Vector3>("scale").z;
  }

  // Create the Bullet trimesh
  for (unsigned int j = 0; j < numIndices; j += 3)
  {
    btVector3 bv0(vertices[indices[j]*3+0],
                  vertices[indices[j]*3+1],
                  vertices[indices[j]*3+2]);

    btVector3 bv1(vertices[indices[j+1]*3+0],
                  vertices[indices[j+1]*3+1],
                  vertices[indices[j+1]*3+2]);

    btVector3 bv2(vertices[indices[j+2]*3+0],
                  vertices[indices[j+2]*3+1],
                  vertices[indices[j+2]*3+2]);

    mTriMesh->addTriangle(bv0, bv1, bv2);
  }

  btConvexShape* convexShape = new btConvexTriangleMeshShape(mTriMesh, true);
  convexShape->setMargin(0.001f);
  bParent->SetCollisionShape(convexShape);

  delete [] vertices;
  delete [] indices;
}
