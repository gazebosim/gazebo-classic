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

#include "transport/Node.hh"
#include "transport/Subscriber.hh"
#include "physics/Model.hh"
#include "physics/World.hh"
#include "physics/Joint.hh"
#include "physics/JointController.hh"

using namespace gazebo;
using namespace physics;

/////////////////////////////////////////////////
JointController::JointController(ModelPtr _model)
  : model(_model)
{
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

  this->jointCmdSub = this->node->Subscribe(std::string("~/") +
      this->model->GetName() + "/joint_cmd",
      &JointController::OnJointCmd, this);
}

/////////////////////////////////////////////////
void JointController::AddJoint(JointPtr _joint)
{
  this->joints[_joint->GetName()] = _joint;
}

/////////////////////////////////////////////////
void JointController::Update()
{
  std::map<std::string, double>::iterator iter;
  for (iter = this->forces.begin(); iter != this->forces.end(); ++iter)
  {
    this->joints[iter->first]->SetForce(0, iter->second);
  }
}

/////////////////////////////////////////////////
void JointController::OnJointCmd(ConstJointCmdPtr &_msg)
{
  std::map<std::string, JointPtr>::iterator iter;
  iter = this->joints.find(_msg->name());
  if (iter != this->joints.end())
    this->forces[_msg->name()] = _msg->force();
  else
    gzerr << "Unable to find joint[" << _msg->name() << "]\n";
}
