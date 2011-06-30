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
/* Desc: ODE Trimesh shape
 * Author: Nate Keonig
 * Date: 16 Oct 2009
 */

#include "common/Mesh.hh"
#include "common/Exception.hh"
#include "common/Console.hh"

#include "physics/ode/ODEGeom.hh"
#include "physics/ode/ODEPhysics.hh"
#include "physics/ode/ODETrimeshShape.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////////////////////////////////
// Constructor
ODETrimeshShape::ODETrimeshShape(GeomPtr parent) : TrimeshShape(parent)
{
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
ODETrimeshShape::~ODETrimeshShape()
{
}

//////////////////////////////////////////////////////////////////////////////
/// Update function.
void ODETrimeshShape::Update()
{
  ODEGeomPtr ogeom = boost::shared_dynamic_cast<ODEGeom>(this->geomParent);

  /// FIXME: use below to update trimesh geometry for collision without using above Ogre codes
  // tell the tri-tri collider the current transform of the trimesh --
  // this is fairly important for good results.

  // Fill in the (4x4) matrix.
  dReal* p_matrix = this->matrix_dblbuff + ( this->last_matrix_index * 16 );
  const dReal *Pos = dGeomGetPosition(ogeom->GetGeomId());
  const dReal *Rot = dGeomGetRotation(ogeom->GetGeomId());

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

  dGeomTriMeshSetLastTransform( ogeom->GetGeomId(),
      *(dMatrix4*)( this->matrix_dblbuff + this->last_matrix_index * 16) );
}

//////////////////////////////////////////////////////////////////////////////
/// Load the trimesh
void ODETrimeshShape::Load( sdf::ElementPtr &_sdf )
{
  TrimeshShape::Load(_sdf);
}

void ODETrimeshShape::Init()
{
  ODEGeomPtr pgeom = boost::shared_static_cast<ODEGeom>(this->geomParent);

  TrimeshShape::Init();

  unsigned int i =0;

  dTriMeshDataID odeData;

  const common::SubMesh *subMesh = this->mesh->GetSubMesh(i);
  if (subMesh->GetVertexCount() < 3)
  {
    gzerr << "ODETrimesh invalid mesh\n";
    return;
  }

  /// This will hold the vertex data of the triangle mesh
  odeData = dGeomTriMeshDataCreate();

  unsigned int numVertices = 0;
  unsigned int numIndices = 0;
  float *vertices = NULL;
  unsigned int *indices = NULL;

  subMesh->FillArrays(&vertices, &indices);

  numIndices = subMesh->GetIndexCount();
  numVertices = subMesh->GetVertexCount();

  for (unsigned int j=0;  j < numVertices; j++)
  {
    vertices[j*3+0] = vertices[j*3+0] * this->sdf->GetValueVector3("scale").x;
    vertices[j*3+1] = vertices[j*3+1] * this->sdf->GetValueVector3("scale").y;
    vertices[j*3+2] = vertices[j*3+2] * this->sdf->GetValueVector3("scale").z;
  }

  // Build the ODE triangle mesh
  dGeomTriMeshDataBuildSingle( odeData,
      (float*)vertices, 3*sizeof(float), numVertices,
      (int*)indices, numIndices, 3*sizeof(int));

  pgeom->SetSpaceId( dSimpleSpaceCreate(pgeom->GetSpaceId()) );
  pgeom->SetGeom( dCreateTriMesh(pgeom->GetSpaceId(), odeData,0,0,0 ), true);

  memset(this->matrix_dblbuff,0,32*sizeof(dReal));
  this->last_matrix_index = 0;
}

