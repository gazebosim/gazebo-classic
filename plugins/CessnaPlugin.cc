/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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

#include <mutex>
#include <string>
#include <sdf/sdf.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include "plugins/CessnaPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(CessnaPlugin)

////////////////////////////////////////////////////////////////////////////////
CessnaPlugin::CessnaPlugin()
{
  this->cmds.fill(0.0f);

  // PID default parameters.
  this->propellerPID.Init(50.0, 0.1, 1, 0.0, 0.0, 20000.0, -20000.0);
  this->propellerPID.SetCmd(0.0);

  for (auto &pid : this->controlSurfacesPID)
  {
    pid.Init(50.0, 0.1, 1, 0.0, 0.0, 20.0, -20.0);
    pid.SetCmd(0.0);
  }
}

/////////////////////////////////////////////////
CessnaPlugin::~CessnaPlugin()
{
  this->updateConnection.reset();
}

/////////////////////////////////////////////////
bool CessnaPlugin::FindJoint(const std::string &_sdfParam, sdf::ElementPtr _sdf,
    physics::JointPtr &_joint)
{
  // Read the required plugin parameters.
  if (!_sdf->HasElement(_sdfParam))
  {
    gzerr << "Unable to find the <" << _sdfParam << "> parameter." << std::endl;
    return false;
  }

  std::string jointName = _sdf->Get<std::string>(_sdfParam);
  _joint = this->model->GetJoint(jointName);
  if (!_joint)
  {
    gzerr << "Failed to find joint [" << jointName
          << "] aborting plugin load." << std::endl;
    return false;
  }
  return true;
}

/////////////////////////////////////////////////
void CessnaPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "CessnaPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "CessnaPlugin _sdf pointer is NULL");
  this->model = _model;

  // Read the required parameter for the propeller max RPMs.
  if (!_sdf->HasElement("propeller_max_rpm"))
  {
    gzerr << "Unable to find the <propeller_max_rpm> parameter." << std::endl;
    return;
  }
  this->propellerMaxRpm = _sdf->Get<int32_t>("propeller_max_rpm");
  if (this->propellerMaxRpm == 0)
  {
    gzerr << "Maximum propeller RPMs cannot be 0" << std::endl;
    return;
  }

  // Read the required joint name parameters.
  std::vector<std::string> requiredParams = {"left_aileron", "left_flap",
    "right_aileron", "right_flap", "elevators", "rudder", "propeller"};

  for (size_t i = 0; i < requiredParams.size(); ++i)
  {
    if (!this->FindJoint(requiredParams[i], _sdf, this->joints[i]))
      return;
  }

  // Overload the PID parameters if they are available.
  if (_sdf->HasElement("propeller_p_gain"))
    this->propellerPID.SetPGain(_sdf->Get<double>("propeller_p_gain"));

  if (_sdf->HasElement("propeller_i_gain"))
    this->propellerPID.SetIGain(_sdf->Get<double>("propeller_i_gain"));

  if (_sdf->HasElement("propeller_d_gain"))
    this->propellerPID.SetDGain(_sdf->Get<double>("propeller_d_gain"));

  if (_sdf->HasElement("surfaces_p_gain"))
  {
    for (auto &pid : this->controlSurfacesPID)
      pid.SetPGain(_sdf->Get<double>("surfaces_p_gain"));
  }

  if (_sdf->HasElement("surfaces_i_gain"))
  {
    for (auto &pid : this->controlSurfacesPID)
      pid.SetIGain(_sdf->Get<double>("surfaces_i_gain"));
  }

  if (_sdf->HasElement("surfaces_d_gain"))
  {
    for (auto &pid : this->controlSurfacesPID)
      pid.SetDGain(_sdf->Get<double>("surfaces_d_gain"));
  }

  // Controller time control.
  this->lastControllerUpdateTime = this->model->GetWorld()->GetSimTime();

  // Listen to the update event. This event is broadcast every simulation
  // iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&CessnaPlugin::Update, this, _1));

  // Initialize transport.
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  std::string prefix = "~/" + this->model->GetName() + "/";
  this->statePub = this->node->Advertise<msgs::Cessna>(prefix + "state");
  this->controlSub = this->node->Subscribe(prefix + "control",
    &CessnaPlugin::OnControl, this);

  gzlog << "Cessna ready to fly. The force will be with you" << std::endl;
}

/////////////////////////////////////////////////
void CessnaPlugin::Update(const common::UpdateInfo &/*_info*/)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  gazebo::common::Time curTime = this->model->GetWorld()->GetSimTime();

  if (curTime > this->lastControllerUpdateTime)
  {
    // Update the control surfaces and publish the new state.
    this->UpdatePIDs((curTime - this->lastControllerUpdateTime).Double());
    this->PublishState();

    this->lastControllerUpdateTime = curTime;
  }
}

/////////////////////////////////////////////////
void CessnaPlugin::OnControl(ConstCessnaPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  if (_msg->has_cmd_propeller_speed() &&
      std::abs(_msg->cmd_propeller_speed()) <= 1)
  {
    this->cmds[kPropeller] = _msg->cmd_propeller_speed();
  }
  if (_msg->has_cmd_left_aileron())
    this->cmds[kLeftAileron] = _msg->cmd_left_aileron();
  if (_msg->has_cmd_left_flap())
    this->cmds[kLeftFlap] = _msg->cmd_left_flap();
  if (_msg->has_cmd_right_aileron())
    this->cmds[kRightAileron] = _msg->cmd_right_aileron();
  if (_msg->has_cmd_right_flap())
    this->cmds[kRightFlap] = _msg->cmd_right_flap();
  if (_msg->has_cmd_elevators())
    this->cmds[kElevators] = _msg->cmd_elevators();
  if (_msg->has_cmd_rudder())
    this->cmds[kRudder] = _msg->cmd_rudder();
}

/////////////////////////////////////////////////
void CessnaPlugin::UpdatePIDs(double _dt)
{
  // Velocity PID for the propeller.
  double vel = this->joints[kPropeller]->GetVelocity(0);
  double maxVel = this->propellerMaxRpm*2.0*M_PI/60.0;
  double target = maxVel * this->cmds[kPropeller];
  double error = vel - target;
  double force = this->propellerPID.Update(error, _dt);
  this->joints[kPropeller]->SetForce(0, force);

  // Position PID for the control surfaces.
  for (size_t i = 0; i < this->controlSurfacesPID.size(); ++i)
  {
    double pos = this->joints[i]->GetAngle(0).Radian();
    error = pos - this->cmds[i];
    force = this->controlSurfacesPID[i].Update(error, _dt);
    this->joints[i]->SetForce(0, force);
  }
}

/////////////////////////////////////////////////
void CessnaPlugin::PublishState()
{
  // Read the current state.
  double propellerRpms = this->joints[kPropeller]->GetVelocity(0)
    /(2.0*M_PI)*60.0;
  float propellerSpeed = propellerRpms / this->propellerMaxRpm;
  float leftAileron = this->joints[kLeftAileron]->GetAngle(0).Radian();
  float leftFlap = this->joints[kLeftFlap]->GetAngle(0).Radian();
  float rightAileron = this->joints[kRightAileron]->GetAngle(0).Radian();
  float rightFlap = this->joints[kRightFlap]->GetAngle(0).Radian();
  float elevators = this->joints[kElevators]->GetAngle(0).Radian();
  float rudder = this->joints[kRudder]->GetAngle(0).Radian();

  msgs::Cessna msg;
  // Set the observed state.
  msg.set_propeller_speed(propellerSpeed);
  msg.set_left_aileron(leftAileron);
  msg.set_left_flap(leftFlap);
  msg.set_right_aileron(rightAileron);
  msg.set_right_flap(rightFlap);
  msg.set_elevators(elevators);
  msg.set_rudder(rudder);

  // Set the target state.
  msg.set_cmd_propeller_speed(this->cmds[kPropeller]);
  msg.set_cmd_left_aileron(this->cmds[kLeftAileron]);
  msg.set_cmd_left_flap(this->cmds[kLeftFlap]);
  msg.set_cmd_right_aileron(this->cmds[kRightAileron]);
  msg.set_cmd_right_flap(this->cmds[kRightFlap]);
  msg.set_cmd_elevators(this->cmds[kElevators]);
  msg.set_cmd_rudder(this->cmds[kRudder]);

  this->statePub->Publish(msg);
}
