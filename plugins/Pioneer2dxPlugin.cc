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
#include "plugins/Pioneer2dxPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(Pioneer2dxPlugin)

/////////////////////////////////////////////////
Pioneer2dxPlugin::Pioneer2dxPlugin()
{
}

/////////////////////////////////////////////////
void Pioneer2dxPlugin::Load(physics::ModelPtr _model,
                            sdf::ElementPtr /*_sdf*/)
{
  this->model = _model;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(model->GetWorld()->GetName());

  this->velSub = this->node->Subscribe(std::string("~/") +
      this->model->GetName() + "/vel_cmd", &Pioneer2dxPlugin::OnVelMsg, this); 

  this->leftJoint = _model->GetJoint("left_wheel_hinge");
  this->rightJoint = _model->GetJoint("right_wheel_hinge");
  this->updateConnection = event::Events::ConnectWorldUpdateStart(
          boost::bind(&Pioneer2dxPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void Pioneer2dxPlugin::OnVelMsg(ConstPosePtr &_msg)
{
  std::cout << "OnVelMsg[" << _msg->DebugString() << "]\n";
}

/////////////////////////////////////////////////
void Pioneer2dxPlugin::OnUpdate()
{
  /*
  this->leftJoint->SetVelocity(0, 0.2);
  this->rightJoint->SetVelocity(0, .2);
  this->leftJoint->SetForce(0, 0.2);
  this->rightJoint->SetForce(0, 0.2);
  */
}
