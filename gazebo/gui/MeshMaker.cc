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

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <sstream>

#include "gazebo/msgs/msgs.hh"

#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/common/Console.hh"

#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/math/Quaternion.hh"
#include "gazebo/common/MouseEvent.hh"

#include "gazebo/transport/Publisher.hh"

#include "gazebo/gui/MeshMaker.hh"

using namespace gazebo;
using namespace gui;

unsigned int MeshMaker::counter = 0;

/////////////////////////////////////////////////
MeshMaker::MeshMaker()
: EntityMaker()
{
  this->visualMsg = new msgs::Visual();
  this->visualMsg->mutable_geometry()->set_type(msgs::Geometry::MESH);
  msgs::Set(this->visualMsg->mutable_pose()->mutable_orientation(),
            ignition::math::Quaterniond());
  msgs::Set(this->visualMsg->mutable_pose()->mutable_position(),
            ignition::math::Vector3d());
}

/////////////////////////////////////////////////
MeshMaker::~MeshMaker()
{
  this->camera.reset();
  delete this->visualMsg;
}

/////////////////////////////////////////////////
void MeshMaker::Init(const std::string &_filename)
{
  this->visualMsg->mutable_geometry()->mutable_mesh()->set_filename(_filename);
}

/////////////////////////////////////////////////
void MeshMaker::Start(const rendering::UserCameraPtr _camera)
{
  this->camera = _camera;

  std::ostringstream stream;
  stream << "__GZ_USER_mesh_" << counter++;
  this->visualMsg->set_name(stream.str());
  this->visualMsg->set_name(stream.str());
  this->visPub->Publish(*this->visualMsg);
}

/////////////////////////////////////////////////
void MeshMaker::Stop()
{
  msgs::Request *msg = msgs::CreateRequest("entity_delete",
                                           this->visualMsg->name());

  this->requestPub->Publish(*msg);
  delete msg;

  gui::Events::moveMode(true);
}

/////////////////////////////////////////////////
void MeshMaker::CreateTheEntity()
{
  msgs::Factory msg;

  std::ostringstream newModelStr;

  newModelStr << "<sdf version ='1.3'>\
    <model name='custom_user_mesh" << counter << "_model'>\
    <pose>" << this->visualMsg->pose().position().x() << " "
              << this->visualMsg->pose().position().y() << " "
              << this->visualMsg->pose().position().z() << " "
              << " 0 0 0</pose>\
    <link name ='link'>\
      <inertial mass ='1.0'>\
          <inertia ixx ='1' ixy ='0' ixz ='0' iyy ='1' iyz ='0' izz ='1'/>\
      </inertial>\
      <collision name ='geom'>\
        <geometry>\
          <mesh filename ='"
          << this->visualMsg->geometry().mesh().filename() << " '/>\
        </geometry>\
      </collision>\
      <visual name ='visual' cast_shadows ='true'>\
        <geometry>\
          <mesh filename ='"
          << this->visualMsg->geometry().mesh().filename() << " '/>\
        </geometry>\
      </visual>\
    </link>\
  </model>\
  </sdf>";

  msg.set_sdf(newModelStr.str());

  msgs::Request *requestMsg = msgs::CreateRequest("entity_delete",
      this->visualMsg->name());
  this->requestPub->Publish(*requestMsg);
  delete requestMsg;

  this->makerPub->Publish(msg);
}
