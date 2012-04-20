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

#include "plugins/JointControllerPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(JointControllerPlugin)

//////////////////////////////////////////////////
JointControllerPlugin::JointControllerPlugin()
{
}

//////////////////////////////////////////////////
void JointControllerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
  this->model = _model;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());
  this->jointCmdSub = this->node->Subscribe(std::string("~/") +
      this->model->GetName() + "/joint_cmd",
      &JointControllerPlugin::OnJointCmd, this);

  for (unsigned int i = 0; i < this->model->GetJointCount(); ++i)
  {
    this->joints.push_back(this->model->GetJoint(i));
  }

  this->updateConnection = event::Events::ConnectWorldUpdateStart(
          boost::bind(&JointControllerPlugin::OnUpdate, this));
}

//////////////////////////////////////////////////
void JointControllerPlugin::Init()
{
}

//////////////////////////////////////////////////
void JointControllerPlugin::OnJointCmd(ConstJointCmdPtr &_msg)
{
  printf("On Joint Command\n");
}

//////////////////////////////////////////////////
void JointControllerPlugin::OnUpdate()
{
  this->joints[0]->SetForce(0, -5.1);
  this->joints[1]->SetForce(0, -5.1);

  this->joints[2]->SetForce(0, 5.1);
  this->joints[3]->SetForce(0, 5.1);

  //this->joints[4]->SetForce(0, 5.5);
}
