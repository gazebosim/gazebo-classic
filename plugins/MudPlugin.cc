/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "plugins/MudPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(MudPlugin)

/////////////////////////////////////////////////
MudPlugin::MudPlugin()
{
}

/////////////////////////////////////////////////
void MudPlugin::Load(physics::ModelPtr _model,
                     sdf::ElementPtr _sdf)
{
  this->model = _model;

  gzerr << "hello\n";

  GZ_ASSERT(_sdf, "MudPlugin _sdf pointer is NULL");
  if (_sdf->HasElement("contactSensorName"))
  {
    this->contactSensorName = _sdf->GetValueString("contactSensorName");

    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(this->model->GetWorld()->GetName());

    std::string topic = std::string("~/") + this->model->GetName() + "/" +
      this->contactSensorName;
    this->contactSub =
      this->node->Subscribe(topic, &MudPlugin::OnContact, this);
  }
  else
  {
    gzerr << "contactSensorName not supplied, ignoring contacts\n";
  }

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&MudPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void MudPlugin::Init()
{
}

/////////////////////////////////////////////////
void MudPlugin::OnContact(ConstContactsPtr &/*_msg*/)
{
  gzerr << "hello\n";
}

/////////////////////////////////////////////////
void MudPlugin::OnUpdate()
{
  common::Time currTime = this->model->GetWorld()->GetSimTime();
  common::Time stepTime = currTime - this->prevUpdateTime;
  this->prevUpdateTime = currTime;

  {
    // ignore everything else, get position and force only
    math::Pose pose;
    //this->model->SetWorldPose(pose);
  }
}
