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
#include "World.hh"
#include "TrimeshShape.hh"
#include "GazeboError.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
TrimeshShape::TrimeshShape(Geom *parent) : Shape(parent)
{
  this->AddType(TRIMESH_SHAPE);

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

  Mass mass = this->geomParent->GetMass();

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
      stream << "  <visual>";
      stream << "    <mesh>" << newName.str() << "</mesh>";
      stream << "    <scale>" << **this->scaleP << "</scale>";
      stream << "  </visual>";
      stream << "</geom:trimesh>";
      stream << "</gazebo:world>";

      XMLConfig *config = new XMLConfig();
      config->LoadString( stream.str() );

      Geom *newGeom = this->GetWorld()->GetPhysicsEngine()->CreateGeom( "trimesh", this->geomParent->GetBody() );

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
 
