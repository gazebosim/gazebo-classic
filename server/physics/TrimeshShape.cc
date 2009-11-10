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
/* Desc: Trimesh shape
 * Author: Nate Keonig
 * Date: 16 Ocy 2009
 * SVN: $Id$
 */

#include "MeshManager.hh"
#include "Mesh.hh"
#include "TrimeshShape.hh"
#include "GazeboError.hh"
#include "OgreAdaptor.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
TrimeshShape::TrimeshShape(Geom *parent) : Shape(parent)
{
  this->type = Shape::TRIMESH;

  Param::Begin(&this->parameters);
  this->meshNameP = new ParamT<std::string>("mesh","",1);
  this->scaleP = new ParamT<Vector3>("scale",Vector3(1,1,1),0);
  Param::End();

  /*this->numVertices = 0;
  this->numIndices = 0;
  this->vertices = NULL;
  this->indices = NULL;
  */
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
TrimeshShape::~TrimeshShape()
{
  delete this->meshNameP;
  delete this->scaleP;
}

//////////////////////////////////////////////////////////////////////////////
/// Load the trimesh
void TrimeshShape::Load(XMLConfigNode *node)
{
  MeshManager *meshManager = MeshManager::Instance();

  this->meshNameP->Load(node);
  this->scaleP->Load(node);

  this->mesh = meshManager->Load( **this->meshNameP );

  /*const Mesh *mesh = meshManager->Load( **this->meshNameP );

  if (!mesh)
    gzthrow("Invalid mesh");

  mesh->FillArrays(&this->vertices, &this->indices);

  this->numIndices = mesh->GetIndexCount();
  this->numVertices = mesh->GetVertexCount();

  for (unsigned int i=0; i < this->numVertices; i++)
  {
    this->vertices[i*3+0] = this->vertices[i*3+0] * (**this->scaleP).x;
    this->vertices[i*3+1] = this->vertices[i*3+1] * (**this->scaleP).y;
    this->vertices[i*3+2] = this->vertices[i*3+2] * (**this->scaleP).z;
  }
  */
}

//////////////////////////////////////////////////////////////////////////////
/// Save child parameters
void TrimeshShape::Save(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(this->meshNameP) << "\n";
  stream << prefix << *(this->scaleP) << "\n";
}
 
