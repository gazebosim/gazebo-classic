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
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/ode/ODECollision.hh"
#include "gazebo/physics/ode/ODEPhysics.hh"
#include "gazebo/physics/ode/ODEMesh.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
ODEMesh::ODEMesh()
{
  this->odeData = nullptr;
  this->vertices = nullptr;
  this->indices = nullptr;
}

//////////////////////////////////////////////////
ODEMesh::~ODEMesh()
{
  delete [] this->vertices;
  delete [] this->indices;
  dGeomTriMeshDataDestroy(this->odeData);
}

//////////////////////////////////////////////////
void ODEMesh::Update()
{
  /// FIXME: use below to update trimesh geometry for collision without
  // using above Ogre codes
  // tell the tri-tri collider the current transform of the trimesh --
  // this is fairly important for good results.

  // Fill in the (4x4) matrix.
  dReal *matrix = this->transform + (this->transformIndex * 16);
  const dReal *Pos = dGeomGetPosition(this->collisionId);
  const dReal *Rot = dGeomGetRotation(this->collisionId);

  matrix[ 0 ] = Rot[ 0 ];
  matrix[ 1 ] = Rot[ 1 ];
  matrix[ 2 ] = Rot[ 2 ];
  matrix[ 3 ] = 0;
  matrix[ 4 ] = Rot[ 4 ];
  matrix[ 5 ] = Rot[ 5 ];
  matrix[ 6 ] = Rot[ 6 ];
  matrix[ 7 ] = 0;
  matrix[ 8 ] = Rot[ 8 ];
  matrix[ 9 ] = Rot[ 9 ];
  matrix[10 ] = Rot[10 ];
  matrix[11 ] = 0;
  matrix[12 ] = Pos[ 0 ];
  matrix[13 ] = Pos[ 1 ];
  matrix[14 ] = Pos[ 2 ];
  matrix[15 ] = 1;

  // Flip to other matrix.
  this->transformIndex = !this->transformIndex;

  dGeomTriMeshSetLastTransform(this->collisionId,
      *reinterpret_cast<dMatrix4*>(this->transform +
                                   this->transformIndex * 16));
}

//////////////////////////////////////////////////
void ODEMesh::Init(const common::SubMesh *_subMesh, ODECollisionPtr _collision,
    const math::Vector3 &_scale)
{
  if (!_subMesh)
    return;

  unsigned int numVertices = _subMesh->GetVertexCount();
  unsigned int numIndices = _subMesh->GetIndexCount();

  this->vertices = nullptr;
  this->indices = nullptr;

  // Get all the vertex and index data
  _subMesh->FillArrays(&this->vertices, &this->indices);

  this->collisionId = _collision->GetCollisionId();

  this->CreateMesh(numVertices, numIndices, _collision, _scale);
}

//////////////////////////////////////////////////
void ODEMesh::Init(const common::Mesh *_mesh, ODECollisionPtr _collision,
    const math::Vector3 &_scale)
{
  if (!_mesh)
    return;

  unsigned int numVertices = _mesh->GetVertexCount();
  unsigned int numIndices = _mesh->GetIndexCount();

  this->vertices = nullptr;
  this->indices = nullptr;

  // Get all the vertex and index data
  _mesh->FillArrays(&this->vertices, &this->indices);

  this->collisionId = _collision->GetCollisionId();
  this->CreateMesh(numVertices, numIndices, _collision, _scale);
}

//////////////////////////////////////////////////
void ODEMesh::CreateMesh(unsigned int _numVertices, unsigned int _numIndices,
    ODECollisionPtr _collision, const math::Vector3 &_scale)
{
  /// This will hold the vertex data of the triangle mesh
  if (this->odeData == nullptr)
    this->odeData = dGeomTriMeshDataCreate();

  // Scale the vertex data
  for (unsigned int j = 0;  j < _numVertices; j++)
  {
    this->vertices[j*3+0] = this->vertices[j*3+0] * _scale.x;
    this->vertices[j*3+1] = this->vertices[j*3+1] * _scale.y;
    this->vertices[j*3+2] = this->vertices[j*3+2] * _scale.z;
  }

  // Build the ODE triangle mesh
  dGeomTriMeshDataBuildSingle(this->odeData,
      this->vertices, 3*sizeof(this->vertices[0]), _numVertices,
      this->indices, _numIndices, 3*sizeof(this->indices[0]));

  if (_collision->GetCollisionId() == nullptr)
  {
    _collision->SetSpaceId(dSimpleSpaceCreate(_collision->GetSpaceId()));
    _collision->SetCollision(dCreateTriMesh(_collision->GetSpaceId(),
          this->odeData, 0, 0, 0), true);
  }
  else
  {
    dGeomTriMeshSetData(_collision->GetCollisionId(), this->odeData);
  }

  memset(this->transform, 0, 32*sizeof(dReal));
  this->transformIndex = 0;
}
