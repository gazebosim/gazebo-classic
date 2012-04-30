/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
 * Author: Nate Keonig
 * Date: 21 May 2009
 */

#include "common/Mesh.hh"

#include "physics/bullet/BulletTypes.hh"
#include "physics/bullet/BulletCollision.hh"
#include "physics/bullet/BulletPhysics.hh"
#include "physics/bullet/BulletTrimeshShape.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletTrimeshShape::BulletTrimeshShape(CollisionPtr _parent)
  : TrimeshShape(_parent)
{
}


//////////////////////////////////////////////////
BulletTrimeshShape::~BulletTrimeshShape()
{
}

//////////////////////////////////////////////////
void BulletTrimeshShape::Update()
{
}

//////////////////////////////////////////////////
void BulletTrimeshShape::Load(sdf::ElementPtr _sdf)
{
  TrimeshShape::Load(_sdf);
}

//////////////////////////////////////////////////
void BulletTrimeshShape::Init()
{
  TrimeshShape::Init();

  BulletCollisionPtr bParent =
    boost::shared_static_cast<BulletCollision>(this->collisionParent);

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
    vertices[j*3+0] = vertices[j*3+0] * this->sdf->GetValueVector3("scale").x;
    vertices[j*3+1] = vertices[j*3+1] * this->sdf->GetValueVector3("scale").y;
    vertices[j*3+2] = vertices[j*3+2] * this->sdf->GetValueVector3("scale").z;
  }

  printf("Num Indices[%d] Vertices[%d]\n", numIndices, numVertices);

  // Create the Bullet trimesh
  for (unsigned int j = 0; j+3 < numIndices; j += 3)
  {
    /*
    printf("J[%d] [%d][%d][%d]\n",j, indices[j], indices[j]+1, indices[j]+2);
    printf("J[%d] [%d][%d][%d]\n",j+1, indices[j+1], indices[j+1]+1,indices[j+1]+2);
    printf("J[%d] [%d][%d][%d]\n",j+2, indices[j+2], indices[j+2]+1,indices[j+2]+2);
    math::Vector3 v0(vertices[indices[j]+0],
                     vertices[indices[j]+1],
                     vertices[indices[j]+2]);

    math::Vector3 v1(vertices[indices[j+1]+0],
                     vertices[indices[j+1]+1],
                     vertices[indices[j+1]+2]);

    math::Vector3 v2(vertices[indices[j+2]+0],
                     vertices[indices[j+2]+1],
                     vertices[indices[j+2]+2]);

    std::cout << "\n-----------------\n";
    std::cout << v0 << "\n" << v1 << "\n" << v2 << "\n--------------------\n";
    */

    btVector3 v0(vertices[indices[j]+0],
               vertices[indices[j]+1],
               vertices[indices[j]+2]);

    btVector3 v1(vertices[indices[j+1]+0],
               vertices[indices[j+1]+1],
               vertices[indices[j+1]+2]);

    btVector3 v2(vertices[indices[j+2]+0],
                     vertices[indices[j+2]+1],
                     vertices[indices[j+2]+2]);

    mTriMesh->addTriangle(v0,v1,v2);
  }

  bParent->SetCollisionShape(new btBvhTriangleMeshShape(mTriMesh, true));
}
