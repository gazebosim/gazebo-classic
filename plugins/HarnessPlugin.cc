/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#include <sdf/sdf.hh>

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Subscriber.hh"

#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/World.hh"

#include "plugins/HarnessPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(HarnessPlugin)

/////////////////////////////////////////////////
HarnessPlugin::HarnessPlugin()
{
}

/////////////////////////////////////////////////
HarnessPlugin::~HarnessPlugin()
{
}

/////////////////////////////////////////////////
void HarnessPlugin::Load(physics::ModelPtr _model,
                         sdf::ElementPtr _sdf)
{
  // Get a pointer to the world
  physics::WorldPtr world = _model->GetWorld();

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(world->GetName());

  this->velocitySub = this->node->Subscribe(
      "~/" + _model->GetName() + "/harness/velocity",
      &HarnessPlugin::OnVelocity, this);

  this->detachSub = this->node->Subscribe(
      "~/" + _model->GetName() + "/harness/detach",
      &HarnessPlugin::OnDetach, this);

  // Load all the harness joints
  sdf::ElementPtr jointElem = _sdf->GetElement("joint");
  while (jointElem)
  {
    std::string jointName = jointElem->Get<std::string>("name");
    try
    {
      auto joint = _model->CreateJoint(jointElem);
      this->joints.push_back(joint);
    }
    catch(gazebo::common::Exception &_e)
    {
      gzerr << "Unable to load joint[" << jointName << "]. "
            << _e.GetErrorStr()
            << std::endl;
    }

    jointElem = jointElem->GetNextElement("joint");
  }

  // Make sure at least one joint was created.
  if (this->joints.empty())
  {
    gzerr << "No joints specified in the harness plugin."
          << "The harness plugin will not run."
          << std::endl;
    return;
  }

  // Get the detach joint
  if (_sdf->HasElement("detach"))
  {
    std::string jointName = _sdf->Get<std::string>("detach");
    this->detachIndex = this->JointIndex(jointName);

    // Error reporting
    if (this->detachIndex < 0)
    {
      this->detachIndex = 0;
      gzwarn << "Invalid <detach> joint name[" << jointName << "] in the "
             << "harness plugin. The first joint will be used as the detach "
             << "joint."
             << std::endl;
    }
  }
  else
  {
    // Error reporting
    gzwarn << "A <detach> element is missing from the harness plugin. "
           << "The first joint will be used as the detach joint."
           << std::endl;
  }

  // Get the winch
  if (_sdf->HasElement("winch"))
  {
    sdf::ElementPtr winchElem = _sdf->GetElement("winch");

    // Find the winch joint
    if (winchElem->HasElement("joint"))
    {
      std::string winchJointName = winchElem->Get<std::string>("joint");
      this->winchIndex = this->JointIndex(winchJointName);

      // Error reporting
      if (this->winchIndex < 0)
      {
        this->winchIndex = 0;
        gzwarn << "Invalid <joint> name[" << winchJointName << "] in the "
               << "<winch> element of the harness plugin.\n"
               << "The first joint will be used as the winch."
               << std::endl;
      }
    }
    else
    {
      // Error reporting
      gzwarn << "A <winch><joint>joint_name</joint></winch> element is "
             << "missing from the harness plugin.\n"
             << "The first joint will be used as the winch."
             << std::endl;
    }

    // Load the Position PID controller
    if (winchElem->HasElement("pos_pid"))
    {
      sdf::ElementPtr pidElem = winchElem->GetElement("pos_pid");
      double pValue = pidElem->HasElement("p") ? pidElem->Get<double>("p") : 0;
      double iValue = pidElem->HasElement("i") ? pidElem->Get<double>("i") : 0;
      double dValue = pidElem->HasElement("d") ? pidElem->Get<double>("d") : 0;
      double iMax =
        pidElem->HasElement("i_max") ? pidElem->Get<double>("i_max") : 0;
      double iMin =
        pidElem->HasElement("i_min") ? pidElem->Get<double>("i_min") : 0;
      double cmdMax =
        pidElem->HasElement("cmd_max") ? pidElem->Get<double>("cmd_max") : 0;
      double cmdMin =
        pidElem->HasElement("cmd_min") ? pidElem->Get<double>("cmd_min") : 0;

      this->winchPosPID.Init(pValue, iValue, dValue,
        iMax, iMin, cmdMax, cmdMin);
    }
    // Load the Velocity PID controller
    if (winchElem->HasElement("vel_pid"))
    {
      sdf::ElementPtr pidElem = winchElem->GetElement("vel_pid");
      double pValue = pidElem->HasElement("p") ? pidElem->Get<double>("p") : 0;
      double iValue = pidElem->HasElement("i") ? pidElem->Get<double>("i") : 0;
      double dValue = pidElem->HasElement("d") ? pidElem->Get<double>("d") : 0;
      double iMax =
        pidElem->HasElement("i_max") ? pidElem->Get<double>("i_max") : 0;
      double iMin =
        pidElem->HasElement("i_min") ? pidElem->Get<double>("i_min") : 0;
      double cmdMax =
        pidElem->HasElement("cmd_max") ? pidElem->Get<double>("cmd_max") : 0;
      double cmdMin =
        pidElem->HasElement("cmd_min") ? pidElem->Get<double>("cmd_min") : 0;

      this->winchVelPID.Init(pValue, iValue, dValue,
        iMax, iMin, cmdMax, cmdMin);
    }
  }
  else
  {
    // Error reporting
    gzwarn << "A <winch> element is missing from the harness plugin. "
           << "The first joint will be used as the winch."
           << std::endl;
  }
}

/////////////////////////////////////////////////
void HarnessPlugin::Init()
{
  for (auto &joint : this->joints)
  {
    try
    {
      joint->Init();
    }
    catch(...)
    {
      gzerr << "Init joint failed" << std::endl;
      return;
    }
  }

  if (!this->joints.empty())
  {
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&HarnessPlugin::OnUpdate, this, std::placeholders::_1));
  }
}

/////////////////////////////////////////////////
void HarnessPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  // Bootstrap the time.
  if (this->prevSimTime == common::Time::Zero)
  {
    this->prevSimTime = _info.simTime;
    return;
  }
  common::Time dt = _info.simTime - this->prevSimTime;

  if (this->winchIndex < 0 ||
      this->winchIndex >= static_cast<int>(this->joints.size()))
  {
    if (this->detachIndex >= 0 &&
        this->detachIndex < static_cast<int>(this->joints.size()))
    {
      gzmsg << "Detaching harness joint" << std::endl;
      this->Detach();
    }
    gzerr << "No known winch joint to control" << std::endl;
    return;
  }

  double pError = 0;
  if (ignition::math::equal(this->winchTargetVel, 0.0f))
  {
    // Calculate the position error if vel target is 0
    pError = this->joints[this->winchIndex]->GetAngle(0).Radian() -
      this->winchTargetPos;
  }

  // Calculate the velocity error
  double vError = this->joints[this->winchIndex]->GetVelocity(0) -
    this->winchTargetVel;


  // Use the PID controller to compute the joint force
  double winchPosForce = this->winchPosPID.Update(pError, dt);
  double winchVelForce = this->winchVelPID.Update(vError, dt);

  // Truncate winchForce so it doesn't push the robot downwards
  // although this can also be accomplished by cmd_min and cmd_max.
  winchVelForce = winchVelForce > 0? winchVelForce : 0.0;

  // Apply the joint force
  this->joints[this->winchIndex]->SetForce(0, winchVelForce + winchPosForce);

  this->prevSimTime = _info.simTime;
}

/////////////////////////////////////////////////
void HarnessPlugin::Detach()
{
  if (this->detachIndex < 0 ||
      this->detachIndex >= static_cast<int>(this->joints.size()))
  {
    gzerr << "No known joint to detach" << std::endl;
    return;
  }
  const auto detachName = this->joints[this->detachIndex]->GetName();
  physics::BasePtr parent = this->joints[this->detachIndex]->Base::GetParent();

  auto model = boost::dynamic_pointer_cast<physics::Model>(parent);
  if (!model)
  {
    gzerr << "Can't get valid model pointer" << std::endl;
    return;
  }

  // We no longer need to update
  this->updateConnection.reset();

  (this->joints[this->detachIndex]).reset();
  model->RemoveJoint(detachName);
  this->detachIndex = -1;
  this->winchIndex = -1;

  this->prevSimTime == common::Time::Zero;
}

/////////////////////////////////////////////////
double HarnessPlugin::WinchVelocity() const
{
  return this->joints[this->winchIndex]->GetVelocity(0);
}

/////////////////////////////////////////////////
void HarnessPlugin::SetWinchVelocity(const float _value)
{
  if (this->winchIndex < 0 ||
      this->winchIndex >= static_cast<int>(this->joints.size()))
  {
    gzerr << "No known winch joint to set velocity" << std::endl;
    return;
  }

  this->winchTargetVel = _value;
  if (ignition::math::equal(_value, 0.0f))
  {
    // if zero velocity is commanded, hold position
    this->winchTargetPos = this->joints[this->winchIndex]->GetAngle(0).Radian();
    this->winchPosPID.Reset();
  }
}

/////////////////////////////////////////////////
int HarnessPlugin::JointIndex(const std::string &_name) const
{
  // Find the winch joint in our list of joints
  for (size_t i = 0; i < this->joints.size(); ++i)
  {
    if (this->joints[i]->GetName() == _name)
      return i;
  }

  return -1;
}

/////////////////////////////////////////////////
void HarnessPlugin::OnVelocity(ConstGzStringPtr &_msg)
{
  try
  {
    this->SetWinchVelocity(std::stof(_msg->data()));
  }
  catch(...)
  {
    gzerr << "Invalid velocity data[" << _msg->data() << "]\n";
  }
}

/////////////////////////////////////////////////
void HarnessPlugin::OnDetach(ConstGzStringPtr &_msg)
{
  if (_msg->data() == "true" ||
      _msg->data() == "TRUE" ||
      _msg->data() == "True")
  {
    this->winchIndex = -1;
  }
}
