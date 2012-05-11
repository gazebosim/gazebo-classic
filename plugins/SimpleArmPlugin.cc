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

#include "physics/physics.h"
#include "transport/transport.h"
#include "plugins/SimpleArmPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(SimpleArmPlugin)

/////////////////////////////////////////////////
SimpleArmPlugin::SimpleArmPlugin()
{
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    this->jointPIDs[i] = common::PID(1, 0.1, 0.01, 1, -1);
    this->jointPositions[i] = 0;
    this->jointVelocities[i] = 0;
    this->jointMaxEfforts[i] = 100;
  }
}

/////////////////////////////////////////////////
void SimpleArmPlugin::Load(physics::ModelPtr _model,
                           sdf::ElementPtr _sdf)
{
  this->model = _model;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

  this->velSub = this->node->Subscribe(std::string("~/") +
      this->model->GetName() + "/simple_arm_joint_cmd",
      &SimpleArmPlugin::OnJointCmdMsg, this);

  if (!_sdf->HasElement("joint1"))
    gzerr << "SimpleArm plugin missing <joint1> element\n";

  // get all joints
  this->joints[0] = _model->GetJoint(
    _sdf->GetElement("joint1")->GetValueString());
  this->jointPIDs[0] = common::PID(
    _sdf->GetElement("joint1_pid")->GetValueVector3().x,
    _sdf->GetElement("joint1_pid")->GetValueVector3().y,
    _sdf->GetElement("joint1_pid")->GetValueVector3().z,
    _sdf->GetElement("joint1_ilim")->GetValueVector2d().y,
    _sdf->GetElement("joint1_ilim")->GetValueVector2d().x);

  this->joints[1] = _model->GetJoint(
    _sdf->GetElement("joint2")->GetValueString());
  this->jointPIDs[1] = common::PID(
    _sdf->GetElement("joint2_pid")->GetValueVector3().x,
    _sdf->GetElement("joint2_pid")->GetValueVector3().y,
    _sdf->GetElement("joint2_pid")->GetValueVector3().z,
    _sdf->GetElement("joint2_ilim")->GetValueVector2d().y,
    _sdf->GetElement("joint2_ilim")->GetValueVector2d().x);

  this->joints[2] = _model->GetJoint(
    _sdf->GetElement("joint3")->GetValueString());
  this->jointPIDs[2] = common::PID(
    _sdf->GetElement("joint3_pid")->GetValueVector3().x,
    _sdf->GetElement("joint3_pid")->GetValueVector3().y,
    _sdf->GetElement("joint3_pid")->GetValueVector3().z,
    _sdf->GetElement("joint3_ilim")->GetValueVector2d().y,
    _sdf->GetElement("joint3_ilim")->GetValueVector2d().x);

  this->joints[3] = _model->GetJoint(
    _sdf->GetElement("joint4")->GetValueString());
  this->jointPIDs[3] = common::PID(
    _sdf->GetElement("joint4_pid")->GetValueVector3().x,
    _sdf->GetElement("joint4_pid")->GetValueVector3().y,
    _sdf->GetElement("joint4_pid")->GetValueVector3().z,
    _sdf->GetElement("joint4_ilim")->GetValueVector2d().y,
    _sdf->GetElement("joint4_ilim")->GetValueVector2d().x);

  this->joints[4] = _model->GetJoint(
    _sdf->GetElement("joint5")->GetValueString());
  this->jointPIDs[4] = common::PID(
    _sdf->GetElement("joint5_pid")->GetValueVector3().x,
    _sdf->GetElement("joint5_pid")->GetValueVector3().y,
    _sdf->GetElement("joint5_pid")->GetValueVector3().z,
    _sdf->GetElement("joint5_ilim")->GetValueVector2d().y,
    _sdf->GetElement("joint5_ilim")->GetValueVector2d().x);


  this->updateConnection = event::Events::ConnectWorldUpdateStart(
          boost::bind(&SimpleArmPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void SimpleArmPlugin::Init()
{
  // physics::EntityPtr parent = boost::shared_dynamic_cast<physics::Entity>(
  //   this->joints[0]->GetChild());
}

/////////////////////////////////////////////////
void SimpleArmPlugin::OnJointCmdMsg(ConstJointCmdPtr &_msg)
{
  // update joint velocities
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    // ignore everything else, get position and force only
    this->jointPositions[i] = _msg->position();
    this->jointMaxEfforts[i] = _msg->force();
  }
}

/////////////////////////////////////////////////
void SimpleArmPlugin::OnUpdate()
{
  common::Time currTime = this->model->GetWorld()->GetSimTime();
  common::Time stepTime = currTime - this->prevUpdateTime;
  this->prevUpdateTime = currTime;

  for (int i = 0; i < NUM_JOINTS; i++)
  {
    // ignore everything else, get position and force only
    double pos_target = this->jointPositions[i];
    double pos_curr = this->joints[i]->GetAngle(0).GetAsRadian();
    double max_cmd = this->jointMaxEfforts[i];

    double pos_err = pos_curr - pos_target;

    double effort_cmd = this->jointPIDs[i].Update(pos_err, stepTime);
    effort_cmd = effort_cmd > max_cmd ? max_cmd :
      (effort_cmd < -max_cmd ? -max_cmd : effort_cmd);
    this->joints[i]->SetForce(0, effort_cmd);
  }
}
