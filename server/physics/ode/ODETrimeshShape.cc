/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: ODE Trimesh shape
 * Author: Nate Keonig
 * Date: 16 Oct 2009
 * SVN: $Id$
 */

#include "Mesh.hh"
#include "GazeboError.hh"
#include "World.hh"
#include "ODEGeom.hh"
#include "ODEPhysics.hh"
#include "ODETrimeshShape.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
ODETrimeshShape::ODETrimeshShape(Geom *parent) : TrimeshShape(parent)
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
  ODEGeom *ogeom = ((ODEGeom*)this->parent);

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
void ODETrimeshShape::Load(XMLConfigNode *node)
{
  dMass odeMass;
  Mass mass;
  ODEGeom *pgeom = (ODEGeom*)this->parent;
  PhysicsEngine *physics = World::Instance()->GetPhysicsEngine();

  TrimeshShape::Load(node);
  /*if (this->mesh->GetSubMeshCount() > 1)
  {
    printf("ODETrimesh submesh count >1\n");
    return;
  }*/

  mass = this->parent->GetMass();

  unsigned int i =0;

  //for (unsigned int i=0; i < this->mesh->GetSubMeshCount(); i++)
  //{
    dTriMeshDataID odeData;

    const SubMesh *subMesh = mesh->GetSubMesh(i);
    if (subMesh->GetVertexCount() < 3)
    {
      printf("ODETrimesh invalid mesh\n");
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
      vertices[j*3+0] = vertices[j*3+0] * (**this->scaleP).x;
      vertices[j*3+1] = vertices[j*3+1] * (**this->scaleP).y;
      vertices[j*3+2] = vertices[j*3+2] * (**this->scaleP).z;
    }

    // Build the ODE triangle mesh
    dGeomTriMeshDataBuildSingle( odeData,
        (float*)vertices, 3*sizeof(float), numVertices,
        (int*)indices, numIndices, 3*sizeof(int));

    pgeom->SetSpaceId( dSimpleSpaceCreate(pgeom->GetSpaceId()) );
    pgeom->SetGeom( dCreateTriMesh(pgeom->GetSpaceId(), odeData,0,0,0 ), true);

    if (!pgeom->IsStatic())
      dMassSetTrimeshTotal(&odeMass, mass.GetAsDouble(), pgeom->GetGeomId());
  //}

  physics->ConvertMass(&mass, &odeMass);
  this->parent->SetMass(mass);

  memset(this->matrix_dblbuff,0,32*sizeof(dReal));
  this->last_matrix_index = 0;
}

