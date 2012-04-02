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

#include "transport/transport.h"
#include "plugins/DiffDrivePlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(DiffDrivePlugin)

/////////////////////////////////////////////////
DiffDrivePlugin::DiffDrivePlugin()
{
}

/////////////////////////////////////////////////
void DiffDrivePlugin::Load(physics::ModelPtr _model,
                            sdf::ElementPtr _sdf)
{
  this->model = _model;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(model->GetWorld()->GetName());

  this->velSub = this->node->Subscribe(std::string("~/") +
      this->model->GetName() + "/vel_cmd", &DiffDrivePlugin::OnVelMsg, this); 

  this->leftJoint = _model->GetJoint(
      _sdf->GetElement("left_joint")->GetValueString());
  this->rightJoint = _model->GetJoint(
      _sdf->GetElement("right_joint")->GetValueString());

  if (!this->leftJoint)
    gzerr << "Unable to find left joint["
          << _sdf->GetElement("left_joint")->GetValueString() << "]\n";
  if (!this->rightJoint)
    gzerr << "Unable to find right joint[" 
          << _sdf->GetElement("right_joint")->GetValueString() << "]\n";

  /*this->updateConnection = event::Events::ConnectWorldUpdateStart(
          boost::bind(&DiffDrivePlugin::OnUpdate, this));
          */
}

/////////////////////////////////////////////////
void DiffDrivePlugin::OnVelMsg(ConstPosePtr &_msg)
{
  std::cout << "OnVelMsg[" << _msg->DebugString() << "]\n";
  this->leftJoint->SetVelocity(0, _msg->position().x());
  this->rightJoint->SetVelocity(0, _msg->position().x());
  this->leftJoint->SetForce(0, 0.2);
  this->rightJoint->SetForce(0, 0.2);
}

/////////////////////////////////////////////////
void DiffDrivePlugin::OnUpdate()
{
  /*
  this->leftJoint->SetVelocity(0, 0.2);
  this->rightJoint->SetVelocity(0, .2);
  this->leftJoint->SetForce(0, 0.2);
  this->rightJoint->SetForce(0, 0.2);
  */
}
