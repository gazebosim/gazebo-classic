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
 * Date: 16 Oct 2009
 */

#include "common/XMLConfig.hh"
#include "common/MeshManager.hh"
#include "common/Mesh.hh"
#include "common/Exception.hh"

#include "physics/World.hh"
#include "physics/PhysicsEngine.hh"
#include "physics/Mass.hh"
#include "physics/Geom.hh"
#include "physics/TrimeshShape.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////////////////////////////////
// Constructor
TrimeshShape::TrimeshShape(GeomPtr parent) 
  : Shape(parent)
{
  this->AddType(Base::TRIMESH_SHAPE);

  common::Param::Begin(&this->parameters);
  this->meshNameP = new common::ParamT<std::string>("filename","",1);
  this->scaleP = new common::ParamT<common::Vector3>("scale",common::Vector3(1,1,1),0);
  this->centerMeshP = new common::ParamT<std::string>("center_mesh","none",0);
  this->genTexCoordP = new common::ParamT<bool>("gen_tex_coord",false,0);
  common::Param::End();
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
void TrimeshShape::Load(common::XMLConfigNode *node)
{
  Shape::Load(node);
  this->meshNameP->Load(node->GetChild("mesh"));
  this->scaleP->Load(node->GetChild("mesh"));
  this->centerMeshP->Load(node);
  this->genTexCoordP->Load(node);
}

//////////////////////////////////////////////////////////////////////////////
/// Init the trimesh shape
void TrimeshShape::Init()
{
  common::MeshManager *meshManager = common::MeshManager::Instance();
  this->mesh = meshManager->Load( **this->meshNameP );

  if (this->centerMeshP->GetValue() == std::string("aabb_center"))
  {
    common::Vector3 center,min_xyz,max_xyz;
    meshManager->GetMeshAABB(this->mesh,center,min_xyz,max_xyz);
    meshManager->SetMeshCenter(this->mesh,center);
  }
  else if (this->centerMeshP->GetValue() == std::string("aabb_bottom"))
  {
    common::Vector3 center,min_xyz,max_xyz;
    meshManager->GetMeshAABB(this->mesh,center,min_xyz,max_xyz);
    meshManager->SetMeshCenter(this->mesh,common::Vector3(center.x,center.y,min_xyz.z));
  }

  if (this->genTexCoordP->GetValue())
  {
    common::Vector3 center,min_xyz,max_xyz;
    meshManager->GetMeshAABB(this->mesh,center,min_xyz,max_xyz);
    meshManager->GenSphericalTexCoord(this->mesh,center);
  }

  Mass mass = this->geomParent->GetMass();

  if (this->mesh->GetSubMeshCount() > 1)
  {
    // Create a mesh for each of the submeshes.
    for (unsigned int i=1; i < this->mesh->GetSubMeshCount(); i++)
    {
      common::SubMesh *subMesh = const_cast<common::SubMesh*>(mesh->GetSubMesh(i));

      if (subMesh->GetVertexCount() < 3)
        continue;

      std::ostringstream newName;
      newName << this->mesh->GetName() << "_" << i;

      common::Mesh *newMesh = new common::Mesh();
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

      common::XMLConfig *config = new common::XMLConfig();
      config->LoadString( stream.str() );

      GeomPtr newGeom = this->GetWorld()->GetPhysicsEngine()->CreateGeom( "trimesh", this->geomParent->GetBody() );

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
