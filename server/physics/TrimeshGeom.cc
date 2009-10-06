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
/* Desc: Trimesh geometry
 * Author: Nate Keonig, Andrew Howard
 * Date: 8 May 2003
 * CVS: $Id$
 */

#include <ode/ode.h>

#include "MeshLoader.hh"
#include "Body.hh"
#include "TrimeshGeom.hh"
#include "GazeboError.hh"
#include "Simulator.hh"
#include "OgreAdaptor.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
TrimeshGeom::TrimeshGeom(Body *body) : Geom(body)
{
  Param::Begin(&this->parameters);
  this->meshNameP = new ParamT<std::string>("mesh","",1);
  this->scaleP = new ParamT<Vector3>("scale",Vector3(1,1,1),0);
  Param::End();
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
TrimeshGeom::~TrimeshGeom()
{
  delete this->meshNameP;
  delete this->scaleP;
}

//////////////////////////////////////////////////////////////////////////////
/// Update function.
void TrimeshGeom::UpdateChild()
{
  /// FIXME: use below to update trimesh geometry for collision without using above Ogre codes
  // tell the tri-tri collider the current transform of the trimesh --
  // this is fairly important for good results.

  // Fill in the (4x4) matrix.
  dReal* p_matrix = this->matrix_dblbuff + ( this->last_matrix_index * 16 );
  const dReal *Pos = dGeomGetPosition(this->geomId);
  const dReal *Rot = dGeomGetRotation(this->geomId);

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

  dGeomTriMeshSetLastTransform( this->geomId,
      *(dMatrix4*)( this->matrix_dblbuff + this->last_matrix_index * 16) );
}

//////////////////////////////////////////////////////////////////////////////
/// Load the trimesh
void TrimeshGeom::LoadChild(XMLConfigNode *node)
{
  MeshLoader *meshLoader;

  unsigned int numVertices = 0;
  unsigned int numIndices = 0;
  float *vertices = NULL;
  unsigned int *indices = NULL;

  this->meshNameP->Load(node);
  this->scaleP->Load(node);

  meshLoader = new MeshLoader();
  meshLoader->Load( this->meshNameP->GetValue() );

  numIndices = meshLoader->GetNumIndices();
  numVertices = meshLoader->GetNumVertices();

  // Create the vertex and index arrays
  vertices = new float[numVertices*3];
  indices = new unsigned int[numIndices];

  meshLoader->FillArrays(&vertices, &indices);

  for (unsigned int i=0; i < numVertices; i++)
  {
    vertices[i*3+0] = vertices[i*3+0] * (**this->scaleP).x;
    vertices[i*3+1] = vertices[i*3+1] * (**this->scaleP).y;
    vertices[i*3+2] = vertices[i*3+2] * (**this->scaleP).z;
  }

  /// This will hold the vertex data of the triangle mesh
  this->odeData = dGeomTriMeshDataCreate();

  // Build the ODE triangle mesh
  dGeomTriMeshDataBuildSingle( this->odeData,
      (float*)vertices, 3*sizeof(float), numVertices,
      (int*)indices, numIndices, 3*sizeof(int));

  this->geomId = dCreateTriMesh( 0,/*this->spaceId,*/ this->odeData,0,0,0 );

  dMassSetTrimesh(&this->mass, this->massP->GetValue(), this->geomId);

  // Create the trimesh geometry
  this->SetGeom(this->geomId, true);

  memset(this->matrix_dblbuff,0,32*sizeof(dReal));
  this->last_matrix_index = 0;

  delete meshLoader;
}

//////////////////////////////////////////////////////////////////////////////
/// Save child parameters
void TrimeshGeom::SaveChild(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(this->meshNameP) << "\n";
  stream << prefix << *(this->scaleP) << "\n";
}
 
