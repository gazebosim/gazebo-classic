/*
 * Copyright 2011 Nate Koenig
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
/* Desc: ODE Trimesh shape
 * Author: Nate Koenig
 * Date: 16 Oct 2009
 */

#include "common/Mesh.hh"
#include "common/Exception.hh"
#include "common/Console.hh"

#include "physics/ode/ODECollision.hh"
#include "physics/ode/ODEPhysics.hh"
#include "physics/ode/ODETrimeshShape.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
ODETrimeshShape::ODETrimeshShape(CollisionPtr _parent) : TrimeshShape(_parent)
{
  this->odeData = NULL;
  this->vertices = NULL;
  this->indices = NULL;
}

//////////////////////////////////////////////////
ODETrimeshShape::~ODETrimeshShape()
{
  delete [] this->vertices;
  delete [] this->indices;
  dGeomTriMeshDataDestroy(this->odeData);
}

//////////////////////////////////////////////////
void ODETrimeshShape::Update()
{
  ODECollisionPtr ocollision =
    boost::shared_dynamic_cast<ODECollision>(this->collisionParent);

  /// FIXME: use below to update trimesh geometry for collision without
  // using above Ogre codes
  // tell the tri-tri collider the current transform of the trimesh --
  // this is fairly important for good results.

  // Fill in the (4x4) matrix.
  dReal* p_matrix = this->matrix_dblbuff + (this->last_matrix_index * 16);
  const dReal *Pos = dGeomGetPosition(ocollision->GetCollisionId());
  const dReal *Rot = dGeomGetRotation(ocollision->GetCollisionId());

  p_matrix[ 0 ] = Rot[ 0 ];
  p_matrix[ 1 ] = Rot[ 1 ];
  p_matrix[ 2 ] = Rot[ 2 ];
  p_matrix[ 3 ] = 0;
  p_matrix[ 4 ] = Rot[ 4 ];
  p_matrix[ 5 ] = Rot[ 5 ];
  p_matrix[ 6 ] = Rot[ 6 ];
  p_matrix[ 7 ] = 0;
  p_matrix[ 8 ] = Rot[ 8 ];
  p_matrix[ 9 ] = Rot[ 9 ];
  p_matrix[10 ] = Rot[10 ];
  p_matrix[11 ] = 0;
  p_matrix[12 ] = Pos[ 0 ];
  p_matrix[13 ] = Pos[ 1 ];
  p_matrix[14 ] = Pos[ 2 ];
  p_matrix[15 ] = 1;

  // Flip to other matrix.
  this->last_matrix_index = !this->last_matrix_index;

  dGeomTriMeshSetLastTransform(ocollision->GetCollisionId(),
      *reinterpret_cast<dMatrix4*>(this->matrix_dblbuff +
                                   this->last_matrix_index * 16));
}

//////////////////////////////////////////////////
void ODETrimeshShape::Load(sdf::ElementPtr _sdf)
{
  TrimeshShape::Load(_sdf);
}

//////////////////////////////////////////////////
void ODETrimeshShape::Init()
{
  TrimeshShape::Init();
  if (!this->mesh)
    return;

  ODECollisionPtr pcollision =
    boost::shared_static_cast<ODECollision>(this->collisionParent);

  /// This will hold the vertex data of the triangle mesh
  if (this->odeData == NULL)
    this->odeData = dGeomTriMeshDataCreate();

  unsigned int numVertices = this->mesh->GetVertexCount();
  unsigned int numIndices = this->mesh->GetIndexCount();
  this->vertices = NULL;
  this->indices = NULL;

  // Get all the vertex and index data
  this->mesh->FillArrays(&this->vertices, &this->indices);

  // Scale the vertex data
  for (unsigned int j = 0;  j < numVertices; j++)
  {
    this->vertices[j*3+0] = this->vertices[j*3+0] *
      this->sdf->GetValueVector3("scale").x;
    this->vertices[j*3+1] = this->vertices[j*3+1] *
      this->sdf->GetValueVector3("scale").y;
    this->vertices[j*3+2] = this->vertices[j*3+2] *
      this->sdf->GetValueVector3("scale").z;
  }

  // Build the ODE triangle mesh
  dGeomTriMeshDataBuildSingle(this->odeData,
      this->vertices, 3*sizeof(this->vertices[0]), numVertices,
      this->indices, numIndices, 3*sizeof(this->indices[0]));

  if (pcollision->GetCollisionId() == NULL)
  {
    pcollision->SetSpaceId(dSimpleSpaceCreate(pcollision->GetSpaceId()));
    pcollision->SetCollision(dCreateTriMesh(pcollision->GetSpaceId(),
          this->odeData, 0, 0, 0), true);
  }
  else
  {
    dGeomTriMeshSetData(pcollision->GetCollisionId(), this->odeData);
  }

  memset(this->matrix_dblbuff, 0, 32*sizeof(dReal));
  this->last_matrix_index = 0;
}
