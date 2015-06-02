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

#include "gazebo/common/Mesh.hh"

#include "gazebo/physics/bullet/BulletTypes.hh"
#include "gazebo/physics/bullet/BulletCollision.hh"
#include "gazebo/physics/bullet/BulletPhysics.hh"
#include "gazebo/physics/bullet/BulletMesh.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletMesh::BulletMesh()
{
}

//////////////////////////////////////////////////
BulletMesh::~BulletMesh()
{
}

//////////////////////////////////////////////////
void BulletMesh::Init(const common::SubMesh *_subMesh,
                      BulletCollisionPtr _collision,
                      const math::Vector3 &_scale)
{
  float *vertices = NULL;
  int *indices = NULL;

  unsigned int numVertices = _subMesh->GetVertexCount();
  unsigned int numIndices = _subMesh->GetIndexCount();

  // Get all the vertex and index data
  _subMesh->FillArrays(&vertices, &indices);

  this->CreateMesh(vertices, indices, numVertices,
                   numIndices, _collision, _scale);

  delete [] vertices;
  delete [] indices;
}

//////////////////////////////////////////////////
void BulletMesh::Init(const common::Mesh *_mesh,
                      BulletCollisionPtr _collision,
                      const math::Vector3 &_scale)
{
  float *vertices = NULL;
  int *indices = NULL;

  unsigned int numVertices = _mesh->GetVertexCount();
  unsigned int numIndices = _mesh->GetIndexCount();

  // Get all the vertex and index data
  _mesh->FillArrays(&vertices, &indices);

  this->CreateMesh(vertices, indices, numVertices,
                   numIndices, _collision, _scale);

  delete [] vertices;
  delete [] indices;
}

/////////////////////////////////////////////////
void BulletMesh::CreateMesh(float *_vertices, int *_indices,
    unsigned int _numVertices, unsigned int _numIndices,
    BulletCollisionPtr _collision, const math::Vector3 &_scale)
{
  btTriangleMesh *mTriMesh = new btTriangleMesh();

  // Scale the vertex data
  for (unsigned int j = 0;  j < _numVertices; ++j)
  {
    _vertices[j*3+0] = _vertices[j*3+0] * _scale.x;
    _vertices[j*3+1] = _vertices[j*3+1] * _scale.y;
    _vertices[j*3+2] = _vertices[j*3+2] * _scale.z;
  }

  // Create the Bullet trimesh
  for (unsigned int j = 0; j < _numIndices; j += 3)
  {
    btVector3 bv0(_vertices[_indices[j]*3+0],
                  _vertices[_indices[j]*3+1],
                  _vertices[_indices[j]*3+2]);

    btVector3 bv1(_vertices[_indices[j+1]*3+0],
                  _vertices[_indices[j+1]*3+1],
                  _vertices[_indices[j+1]*3+2]);

    btVector3 bv2(_vertices[_indices[j+2]*3+0],
                  _vertices[_indices[j+2]*3+1],
                  _vertices[_indices[j+2]*3+2]);

    mTriMesh->addTriangle(bv0, bv1, bv2);
  }

  btGImpactMeshShape *gimpactMeshShape =
    new btGImpactMeshShape(mTriMesh);
  gimpactMeshShape->updateBound();

  _collision->SetCollisionShape(gimpactMeshShape);
}
