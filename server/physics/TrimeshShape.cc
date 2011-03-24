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

#include "PhysicsEngine.hh"
#include "MeshManager.hh"
#include "Mesh.hh"
#include "Mass.hh"
#include "Geom.hh"
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
  this->centerMeshP = new ParamT<std::string>("centerMesh","none",0);
  this->genTexCoordP = new ParamT<bool>("genTexCoord",false,0);
  Param::End();
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
TrimeshShape::~TrimeshShape()
{
  delete this->meshNameP;
  delete this->scaleP;
  delete this->centerMeshP;
  delete this->genTexCoordP;
}

//////////////////////////////////////////////////////////////////////////////
/// Load the trimesh
void TrimeshShape::Load(XMLConfigNode *node)
{
  MeshManager *meshManager = MeshManager::Instance();

  this->meshNameP->Load(node);
  this->scaleP->Load(node);
  this->centerMeshP->Load(node);
  this->genTexCoordP->Load(node);
 
  this->mesh = meshManager->Load( **this->meshNameP );
  //this->mesh->Print();
  
  if (this->centerMeshP->GetValue() == std::string("aabb_center"))
  {
    Vector3 center,min_xyz,max_xyz;
    meshManager->GetMeshAABB(this->mesh,center,min_xyz,max_xyz);
    meshManager->SetMeshCenter(this->mesh,center);
    //std::cout << " aabb center " << std::endl;
  }
  else if (this->centerMeshP->GetValue() == std::string("aabb_bottom"))
  {
    Vector3 center,min_xyz,max_xyz;
    meshManager->GetMeshAABB(this->mesh,center,min_xyz,max_xyz);
    meshManager->SetMeshCenter(this->mesh,Vector3(center.x,center.y,min_xyz.z));
    //std::cout << " aabb bottom " << std::endl;
  }

  if (this->genTexCoordP->GetValue())
  {
    Vector3 center,min_xyz,max_xyz;
    meshManager->GetMeshAABB(this->mesh,center,min_xyz,max_xyz);
    meshManager->GenSphericalTexCoord(this->mesh,center);
  }

  Mass mass = this->parent->GetMass();

  if (this->mesh->GetSubMeshCount() > 1)
  {
    // Create a mesh for each of the submeshes.
    for (unsigned int i=1; i < this->mesh->GetSubMeshCount(); i++)
    {
      SubMesh *subMesh = const_cast<SubMesh*>(mesh->GetSubMesh(i));

      if (subMesh->GetVertexCount() < 3)
        continue;

      std::ostringstream newName;
      newName << this->mesh->GetName() << "_" << i;

      Mesh *newMesh = new Mesh();
      newMesh->SetName( newName.str() );
      newMesh->AddSubMesh( subMesh );
     
      meshManager->AddMesh( newMesh ); 

      std::ostringstream stream;

      stream << "<gazebo:world xmlns:gazebo=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#gz\" xmlns:geom=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#geom\">"; 

      stream << "<geom:trimesh name='" << newName.str() << "_geom'>";
      stream << "  <mass>" << 
        mass.GetAsDouble() / this->mesh->GetSubMeshCount() << "</mass>";
      stream << "  <xyz>0 0 0</xyz>";
      stream << "  <scale>" << **this->scaleP << "</scale>";
      stream << "  <mesh>" << newName.str() << "</mesh>";
      stream << "</geom:trimesh>";
      stream << "</gazebo:world>";

      XMLConfig *config = new XMLConfig();
      config->LoadString( stream.str() );

      Geom *newGeom = this->physicsEngine->CreateGeom( "trimesh", 
          this->parent->GetBody() );

      newGeom->SetSaveable(false);
      newGeom->Load( config->GetRootNode()->GetChild() );

      delete config;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
/// Save child parameters
void TrimeshShape::Save(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(this->meshNameP) << "\n";
  stream << prefix << *(this->scaleP) << "\n";
}
 
