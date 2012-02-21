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

#include "common/MeshManager.hh"
#include "common/Mesh.hh"
#include "common/Exception.hh"

#include "physics/World.hh"
#include "physics/PhysicsEngine.hh"
#include "physics/Collision.hh"
#include "physics/TrimeshShape.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
TrimeshShape::TrimeshShape(CollisionPtr _parent)
  : Shape(_parent)
{
  this->AddType(Base::TRIMESH_SHAPE);
}


//////////////////////////////////////////////////
TrimeshShape::~TrimeshShape()
{
}

//////////////////////////////////////////////////
void TrimeshShape::Init()
{
  common::MeshManager *meshManager = common::MeshManager::Instance();
  this->mesh = meshManager->Load(this->sdf->GetValueString("filename"));

  if (this->mesh->GetSubMeshCount() > 1)
  {
    // Create a mesh for each of the submeshes.
    for (unsigned int i = 1; i < this->mesh->GetSubMeshCount(); i++)
    {
      common::SubMesh *subMesh =
        const_cast<common::SubMesh*>(mesh->GetSubMesh(i));

      if (subMesh->GetVertexCount() < 3)
        continue;

      std::ostringstream newName;
      newName << this->mesh->GetName() << "_" << i;

      common::Mesh *newMesh = new common::Mesh();
      newMesh->SetName(newName.str());
      newMesh->AddSubMesh(subMesh);

      meshManager->AddMesh(newMesh);

      std::ostringstream stream;

      stream << "<collision name ='" << newName.str() << "_collision'>";
      stream << "  <geometry>";
      stream << "    <mesh filename ='" << newName.str() << "' scale ='"
             << this->sdf->GetValueVector3("scale") << "'/>";
      stream << "  </geometry>";
      stream << "</collision>";
      stream << "<visual name ='" << newName.str() << "_visual'>";
      stream << "  <geometry>";
      stream << "    <mesh filename ='" << newName.str() << "' scale ='"
             << this->sdf->GetValueVector3("scale") << "'/>";
      stream << "  </geometry>";
      stream << "</visual>";

      CollisionPtr newCollision =
        this->GetWorld()->GetPhysicsEngine()->CreateCollision(
          "trimesh", this->collisionParent->GetLink());

      // TODO: need to implement this function properly.
      newCollision->SetSaveable(false);
      // newCollision->Load(config->GetRootNode()->GetChild());
    }
  }
}

//////////////////////////////////////////////////
void TrimeshShape::SetScale(const math::Vector3 &_scale)
{
  this->sdf->GetAttribute("scale")->Set(_scale);
}

//////////////////////////////////////////////////
math::Vector3 TrimeshShape::GetSize() const
{
  return this->sdf->GetValueVector3("scale");
}

//////////////////////////////////////////////////
std::string TrimeshShape::GetFilename() const
{
  return this->sdf->GetValueString("filename");
}

//////////////////////////////////////////////////
void TrimeshShape::SetFilename(const std::string &_filename)
{
  this->sdf->GetAttribute("filename")->Set(_filename);
  this->Init();
}

//////////////////////////////////////////////////
void TrimeshShape::FillShapeMsg(msgs::Geometry &_msg)
{
  _msg.set_type(msgs::Geometry::MESH);
  _msg.mutable_mesh()->set_filename(this->GetFilename());
  msgs::Set(_msg.mutable_mesh()->mutable_scale(), this->GetSize());
}

//////////////////////////////////////////////////
void TrimeshShape::ProcessMsg(const msgs::Geometry &_msg)
{
  this->SetScale(msgs::Convert(_msg.mesh().scale()));
  this->SetFilename(_msg.mesh().filename());
}


