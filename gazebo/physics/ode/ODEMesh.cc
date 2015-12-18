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
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/ode/ODECollision.hh"
#include "gazebo/physics/ode/ODEPhysics.hh"

#include "gazebo/physics/ode/ODEMeshPrivate.hh"
#include "gazebo/physics/ode/ODEMesh.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
ODEMesh::ODEMesh()
: dataPtr(new ODEMeshPrivate)
{
  this->dataPtr->odeData = NULL;
  this->dataPtr->vertices = NULL;
  this->dataPtr->indices = NULL;
}

//////////////////////////////////////////////////
ODEMesh::~ODEMesh()
{
  delete [] this->dataPtr->vertices;
  delete [] this->dataPtr->indices;
  dGeomTriMeshDataDestroy(this->dataPtr->odeData);
}

//////////////////////////////////////////////////
void ODEMesh::Update()
{
  /// FIXME: use below to update trimesh geometry for collision without
  // using above Ogre codes
  // tell the tri-tri collider the current transform of the trimesh --
  // this is fairly important for good results.

  // Fill in the (4x4) matrix.
  dReal *matrix = this->dataPtr->transform +
    (this->dataPtr->transformIndex * 16);
  const dReal *pos = dGeomGetPosition(this->dataPtr->collisionId);
  const dReal *rot = dGeomGetRotation(this->dataPtr->collisionId);

  matrix[ 0] = rot[0];
  matrix[ 1] = rot[1];
  matrix[ 2] = rot[2];
  matrix[ 3] = 0;
  matrix[ 4] = rot[4];
  matrix[ 5] = rot[5];
  matrix[ 6] = rot[6];
  matrix[ 7] = 0;
  matrix[ 8] = rot[8];
  matrix[ 9] = rot[9];
  matrix[10] = rot[0];
  matrix[11] = 0;
  matrix[12] = pos[0];
  matrix[13] = pos[1];
  matrix[14] = pos[2];
  matrix[15] = 1;

  // Flip to other matrix.
  this->dataPtr->transformIndex = !this->dataPtr->transformIndex;

  dGeomTriMeshSetLastTransform(this->dataPtr->collisionId,
      *reinterpret_cast<dMatrix4*>(this->dataPtr->transform +
                                   this->dataPtr->transformIndex * 16));
}

//////////////////////////////////////////////////
void ODEMesh::Init(const common::SubMesh *_subMesh,
    ODECollisionPtr _collision,
    const math::Vector3 &_scale)
{
  this->Init(_subMesh, _collision, _scale.Ign());
}

//////////////////////////////////////////////////
void ODEMesh::Init(const common::SubMesh *_subMesh,
    ODECollisionPtr _collision,
    const ignition::math::Vector3d &_scale)
{
  if (!_subMesh)
    return;

  unsigned int numVertices = _subMesh->GetVertexCount();
  unsigned int numIndices = _subMesh->GetIndexCount();

  this->dataPtr->vertices = NULL;
  this->dataPtr->indices = NULL;

  // Get all the vertex and index data
  _subMesh->FillArrays(&this->dataPtr->vertices, &this->dataPtr->indices);

  this->dataPtr->collisionId = _collision->CollisionId();

  this->CreateMesh(numVertices, numIndices, _collision, _scale);
}

//////////////////////////////////////////////////
void ODEMesh::Init(const common::Mesh *_mesh, ODECollisionPtr _collision,
    const math::Vector3 &_scale)
{
  this->Init(_mesh, _collision, _scale.Ign());
}

//////////////////////////////////////////////////
void ODEMesh::Init(const common::Mesh *_mesh, ODECollisionPtr _collision,
    const ignition::math::Vector3d &_scale)
{
  if (!_mesh)
    return;

  unsigned int numVertices = _mesh->GetVertexCount();
  unsigned int numIndices = _mesh->GetIndexCount();

  this->dataPtr->vertices = NULL;
  this->dataPtr->indices = NULL;

  // Get all the vertex and index data
  _mesh->FillArrays(&this->dataPtr->vertices, &this->dataPtr->indices);

  this->dataPtr->collisionId = _collision->CollisionId();
  this->CreateMesh(numVertices, numIndices, _collision, _scale);
}

//////////////////////////////////////////////////
void ODEMesh::CreateMesh(const unsigned int _numVertices,
    const unsigned int _numIndices,
    ODECollisionPtr _collision,
    const ignition::math::Vector3d &_scale)
{
  /// This will hold the vertex data of the triangle mesh
  if (this->dataPtr->odeData == NULL)
    this->dataPtr->odeData = dGeomTriMeshDataCreate();

  // Scale the vertex data
  for (unsigned int j = 0;  j < _numVertices; ++j)
  {
    this->dataPtr->vertices[j*3+0] = this->dataPtr->vertices[j*3+0] *
      _scale.X();
    this->dataPtr->vertices[j*3+1] = this->dataPtr->vertices[j*3+1] *
      _scale.Y();
    this->dataPtr->vertices[j*3+2] = this->dataPtr->vertices[j*3+2] *
      _scale.Z();
  }

  // Build the ODE triangle mesh
  dGeomTriMeshDataBuildSingle(this->dataPtr->odeData,
      this->dataPtr->vertices, 3*sizeof(this->dataPtr->vertices[0]),
      _numVertices, this->dataPtr->indices, _numIndices,
      3*sizeof(this->dataPtr->indices[0]));

  if (_collision->CollisionId() == NULL)
  {
    _collision->SetSpaceId(dSimpleSpaceCreate(_collision->SpaceId()));
    _collision->SetCollision(dCreateTriMesh(_collision->SpaceId(),
          this->dataPtr->odeData, 0, 0, 0), true);
  }
  else
  {
    dGeomTriMeshSetData(_collision->CollisionId(), this->dataPtr->odeData);
  }

  memset(this->dataPtr->transform, 0, 32*sizeof(dReal));
  this->dataPtr->transformIndex = 0;
}
