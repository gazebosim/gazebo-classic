/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/MeshManager.hh"
#include "gazebo/common/Mesh.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/World.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/MeshShape.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
MeshShape::MeshShape(CollisionPtr _parent)
  : Shape(_parent)
{
  this->submesh = NULL;
  this->AddType(Base::MESH_SHAPE);
  sdf::initFile("mesh_shape.sdf", this->sdf);
}

//////////////////////////////////////////////////
MeshShape::~MeshShape()
{
}

//////////////////////////////////////////////////
void MeshShape::Init()
{
  std::string meshStr = this->sdf->Get<std::string>("uri");

  common::MeshManager *meshManager = common::MeshManager::Instance();
  this->mesh = meshManager->GetMesh(meshStr);

  if (!this->mesh)
  {
    meshStr = common::find_file(this->sdf->Get<std::string>("uri"));

    if (meshStr == "__default__" || meshStr.empty())
    {
      gzerr << "No mesh specified\n";
      return;
    }

    if ((this->mesh = meshManager->Load(meshStr)) == NULL)
      gzerr << "Unable to load mesh from file[" << meshStr << "]\n";
  }

  if (this->submesh)
    delete this->submesh;
  this->submesh = NULL;

  if (this->sdf->HasElement("submesh"))
  {
    sdf::ElementPtr submeshElem = this->sdf->GetElement("submesh");
    std::string submeshName = submeshElem->Get<std::string>("name");
    if (submeshName != "__default__" && !submeshName.empty())
    {
      const common::SubMesh *smesh = this->mesh->GetSubMesh(submeshName);
      if (smesh)
      {
        this->submesh = new common::SubMesh(
          this->mesh->GetSubMesh(submeshName));

        if (!this->submesh)
          gzthrow("Unable to get submesh with name[" +
              submeshElem->Get<std::string>("name") + "]");

        // Center the submesh if specified in SDF.
        if (submeshElem->HasElement("center") &&
            submeshElem->Get<bool>("center"))
        {
          this->submesh->Center();
        }
      }
    }
  }
}

//////////////////////////////////////////////////
void MeshShape::SetScale(const math::Vector3 &_scale)
{
  this->sdf->GetElement("scale")->Set(_scale);
}

//////////////////////////////////////////////////
math::Vector3 MeshShape::GetSize() const
{
  return this->sdf->Get<math::Vector3>("scale");
}

//////////////////////////////////////////////////
std::string MeshShape::GetMeshURI() const
{
  return this->sdf->Get<std::string>("uri");
}

//////////////////////////////////////////////////
void MeshShape::SetMesh(const std::string &_uri,
                           const std::string &_submesh,
                           bool _center)
{
  this->sdf->GetElement("uri")->Set(_uri);

  if (!_submesh.empty())
  {
    this->sdf->GetElement("submesh")->GetElement("name")->Set(_submesh);
    this->sdf->GetElement("submesh")->GetElement("center")->Set(_center);
  }

  this->Init();
}

//////////////////////////////////////////////////
void MeshShape::FillMsg(msgs::Geometry &_msg)
{
  _msg.set_type(msgs::Geometry::MESH);
  _msg.mutable_mesh()->CopyFrom(msgs::MeshFromSDF(this->sdf));
}

//////////////////////////////////////////////////
void MeshShape::ProcessMsg(const msgs::Geometry &_msg)
{
  this->SetScale(msgs::Convert(_msg.mesh().scale()));
  this->SetMesh(_msg.mesh().filename(),
      _msg.mesh().has_submesh() ? _msg.mesh().submesh() : std::string(),
      _msg.mesh().has_center_submesh() ? _msg.mesh().center_submesh() :  false);
}
