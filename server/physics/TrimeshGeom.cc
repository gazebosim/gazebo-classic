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
#include <Ogre.h>

#include "OgreVisual.hh"
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
  Ogre::SubMesh* subMesh;
  Ogre::MeshPtr mesh;

  unsigned int i,j;
  unsigned int numVertices = 0;
  unsigned int numIndices = 0;
  const Ogre::VertexElement* elem;
  Ogre::VertexData *vertexData;
  Ogre::HardwareVertexBufferSharedPtr vbuf;
  Ogre::HardwareIndexBufferSharedPtr ibuf;
  unsigned short* indTmp = NULL;
  unsigned char* pData = NULL;
  float *pFloat = NULL;
  float *vertPtr = NULL;
  float *vertices = NULL;
  int *indices = NULL;
  int vindex = 0;
  int iindex = 0;

  this->meshNameP->Load(node);
  this->scaleP->Load(node);

  mesh = Ogre::MeshManager::getSingleton().load(this->meshNameP->GetValue(),Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  if (mesh.isNull())
    gzthrow("Failed to load trimesh "+ this->meshNameP->GetValue());

  // Count the number of vertices and indices
  // for (i=0; i<mesh->getNumSubMeshes(); i++)
  for (i=0; i<1; i++)
  {
    subMesh = mesh->getSubMesh(i);
    if (subMesh->useSharedVertices)
      numVertices += mesh->sharedVertexData->vertexCount;
    else
      numVertices += subMesh->vertexData->vertexCount;
    numIndices += subMesh->indexData->indexCount;
  }

  // Create the vertex and index arrays
  vertices = new float[numVertices*3];
  indices = new int[numIndices];


  // Copy the vertex and index data from OGRE
  //for (i = 0; i < mesh->getNumSubMeshes(); i++)
  for (i = 0; i < 1; i++)
  {
    subMesh = mesh->getSubMesh(i);

    if (subMesh->useSharedVertices)
      vertexData=mesh->sharedVertexData;
    else
      vertexData=subMesh->vertexData;

    elem = vertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
    vbuf = vertexData->vertexBufferBinding->getBuffer(elem->getSource());

    // Pointer to vertex array.
    vertPtr = &(vertices[vindex]);
    vindex += vertexData->vertexCount*3;

    pData = static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

    // Add vertices to the vertex array
    for (j = 0; j < vertexData->vertexCount; j++)
    {
      elem->baseVertexPointerToElement(pData, &pFloat);

      *(vertPtr+0) = (*pFloat++) * this->scaleP->GetValue().x;
      *(vertPtr+1) = (*pFloat++) * this->scaleP->GetValue().y;
      *(vertPtr+2) = (*pFloat++) * this->scaleP->GetValue().z;

      vertPtr += 3;

      pData += vbuf->getVertexSize();
    }

    vbuf->unlock();

    // Get the indices
    ibuf = subMesh->indexData->indexBuffer;
    indTmp = new unsigned short[subMesh->indexData->indexCount];

    ibuf->readData(0, ibuf->getSizeInBytes(), indTmp);

    /// Copy the indices
    for (j = 0; j < subMesh->indexData->indexCount; j++)
    {
      indices[j+iindex] = indTmp[j];
    }

    iindex += subMesh->indexData->indexCount;

    if (indTmp)
    {
      delete [] indTmp;
      indTmp = NULL;
    }
  }


  /// This will hold the vertex data of the triangle mesh
  this->odeData = dGeomTriMeshDataCreate();

  // Build the ODE triangle mesh
  dGeomTriMeshDataBuildSingle( this->odeData,
                               (float*)vertices, 3*sizeof(float), numVertices,
                               (int*)indices, numIndices, 3*sizeof(int));

  this->geomId = dCreateTriMesh( this->spaceId, this->odeData,0,0,0 );

  dMassSetTrimesh(&this->mass, this->massP->GetValue(), this->geomId);

  // Create the trimesh geometry
  this->SetGeom(this->geomId, true);

  memset(this->matrix_dblbuff,0,32*sizeof(dReal));
  this->last_matrix_index = 0;
}

//////////////////////////////////////////////////////////////////////////////
/// Save child parameters
void TrimeshGeom::SaveChild(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(this->meshNameP) << "\n";
  stream << prefix << *(this->scaleP) << "\n";
}
 
