/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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

#include "gazebo/physics/ContactManager.hh"
#include "gazebo/common/MeshCSG.hh"
#include "plugins/DrillPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(DrillPlugin)

/////////////////////////////////////////////////
DrillPlugin::DrillPlugin()
{
}

/////////////////////////////////////////////////
void DrillPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model = _model;
  this->world = this->model->GetWorld();

  if (!_sdf->HasElement("drill_bit_collision"))
  {
    gzerr << "Drill plugin missing <drill_bit_collision> element\n";
    return;
  }

  this->drillBit = _model->GetChildCollision(
        _sdf->GetElement("drill_bit_collision")->Get<std::string>());

  if (!this->drillBit)
  {
    gzerr << "Unable to find drill_bit_collision["
          << _sdf->GetElement("drill_bit_collision")->Get<std::string>()
          << "]\n";
    return;
  }

  // \todo: Test with multiple drills, the name here are hardcoded...
  std::string bitMeshName = "drill_bit_shape_mesh";

  physics::ShapePtr shape = this->drillBit->GetShape();
  if (shape->HasType(physics::Base::CYLINDER_SHAPE))
  {
    physics::CylinderShapePtr bitShape =
      boost::dynamic_pointer_cast<physics::CylinderShape>(shape);

    std::cout << "Radius[" << bitShape->GetRadius() << "] Len[" << 
bitShape->GetLength() << "]\n";

    common::MeshManager::Instance()->CreateCylinder(bitMeshName,
        bitShape->GetRadius(), bitShape->GetLength(), 2, 32);
  }
  else if (shape->HasType(physics::Base::SPHERE_SHAPE))
  {
    physics::SphereShapePtr bitShape =
      boost::dynamic_pointer_cast<physics::SphereShape>(shape);
    common::MeshManager::Instance()->CreateSphere(bitMeshName,
        bitShape->GetRadius(), 32, 32);
  }
  else if (shape->HasType(physics::Base::BOX_SHAPE))
  {
    physics::BoxShapePtr bitShape =
      boost::dynamic_pointer_cast<physics::BoxShape>(shape);
    common::MeshManager::Instance()->CreateBox(bitMeshName,
        bitShape->GetSize(), math::Vector2d(0,0));
  }
  else
  {
    gzerr << "A drill geometry must be a cylinder, box, or sphere.\n";
    return;
  }

  this->bitMesh = common::MeshManager::Instance()->GetMesh(bitMeshName);
  if (!this->bitMesh)
  {
    gzerr << "Unable to create drill bit mesh\n";
    return;
  }

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->world->GetName());

  this->visPub = this->node->Advertise<msgs::Visual>("~/visual");

  physics::ContactManager *mgr =
    this->model->GetWorld()->GetPhysicsEngine()->GetContactManager();
  std::string topic = mgr->CreateFilter("drill",
      this->drillBit->GetScopedName());

  if (!this->contactSub)
  {
    this->contactSub = this->node->Subscribe(topic,
        &DrillPlugin::OnContacts, this);
  }

  // this->updateConnection = event::Events::ConnectWorldUpdateBegin(
  //       boost::bind(&DrillPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void DrillPlugin::Init()
{
}

/////////////////////////////////////////////////
void DrillPlugin::OnUpdate()
{
}

//////////////////////////////////////////////////
void DrillPlugin::OnContacts(ConstContactsPtr &_msg)
{
  //for (ContactMsgs_L::iterator iter = this->incomingContacts.begin();
  //    iter != this->incomingContacts.end(); ++iter)
  //{
    // Iterate over all the contacts in the message
    for (int i = 0; i < _msg->contact_size(); ++i)
    {
      if (_msg->contact(i).depth(0) < 0.1)
        continue;
      std::string collision1 = _msg->contact(i).collision1();
      std::string collision2 = _msg->contact(i).collision2();

      std::string pieceName;
      if (collision1 == this->drillBit->GetScopedName())
        pieceName = collision2;
      else
        pieceName = collision1;

      physics::EntityPtr pieceEntity = this->world->GetEntity(pieceName);
      if (!pieceEntity)
      {
        gzerr << "Unable to find entity to cut with name["
          << pieceName << "]\n";
        continue;
      }

      physics::CollisionPtr pieceColl =
        boost::dynamic_pointer_cast<physics::Collision>(pieceEntity);
      physics::MeshShapePtr pieceShape = 
        boost::dynamic_pointer_cast<physics::MeshShape>(pieceColl->GetShape());

      common::MeshCSG meshCSG;

      common::Mesh *booleanMesh = meshCSG.CreateBoolean(
          pieceShape->GetMesh(),
          this->bitMesh,
          common::MeshCSG::DIFFERENCE);


      msgs::Visual msg;

      msg.set_name("default::flat_box::link::visual");
      msg.set_parent_name("default::flat_box::link");
      msgs::Mesh *meshMsg = msg.mutable_geometry()->mutable_triangleMesh();
      for (unsigned int i = 0; i < booleanMesh->GetSubMeshCount(); ++i)
      {
        common::SubMesh *subMesh = booleanMesh->GetSubMesh(i);
        msgs::Mesh::SubMesh *subMeshMsg = meshMesh->add_subMesh();
        for (unsigned int j = 0; j < subMesh->GetVertexCount(); ++j)
          msgs::Set(subMeshMsg->add_vertex(), subMesh->GetVertex(j));
        for (unsigned int j = 0; j < subMesh->GetNormalCount(); ++j)
          msgs::Set(subMeshMsg->add_normal(), subMesh->GetVertex(j));
        for (unsigned int j = 0; j < subMesh->GetIndexCount(); ++j)
          subMeshMsg->add_index(subMesh->GetIndex(j));
      }
      msg.mutable_scale()->set_x(2.0);
      this->visPub->Publish(msg);

      std::cout << "Piece[" << pieceColl->GetScopedName() << "]\n";
    }
  //}
}
