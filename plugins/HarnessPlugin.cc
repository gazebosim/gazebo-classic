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

#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/Joint.hh"
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
  if (!_sdf->HasElement("harness_link_name"))
  {
    gzerr << "No <harness_link_name> present. Harness plugin will not run.\n";
    return;
  }
  std::string harnessLinkName = _sdf->Get<std::string>("harness_link_name");

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

  // Create a link that is fixed the world. This will act as the base from
  // which a model can be lowered
  this->harnessLink = _model->CreateLink(harnessLinkName);
  this->harnessLink->SetWorldPose(math::Pose(0, 0, 1, 0, 0, 0));

  this->harnessJoint = world->GetPhysicsEngine()->CreateJoint("fixed");
  this->harnessJoint->SetName(harnessLinkName + "__fixed_joint__");
  this->harnessJoint->Attach(physics::LinkPtr(), this->harnessLink);
  this->harnessJoint->Load(physics::LinkPtr(), this->harnessLink,
      ignition::math::Pose3d::Zero);

  // Load all the harness joints
  sdf::ElementPtr jointElem = _sdf->GetElement("joint");
  while (jointElem)
  {
    std::string jointName = jointElem->Get<std::string>("name");
    std::string jointType = jointElem->Get<std::string>("type");

    // Create the joint
    physics::JointPtr joint = world->GetPhysicsEngine()->CreateJoint(jointType);
    joint->SetModel(_model);
    if (joint)
    {
      // Load the joint
      try
      {
        joint->SetWorld(world);

        joint->Load(jointElem);
        this->joints.push_back(joint);
      }
      catch(gazebo::common::Exception &_e)
      {
        gzerr << "Unable to load joint[" << jointName << "]. "
          << _e.GetErrorStr() << "]\n";
      }
    }
    else
    {
      gzerr << "Unable to create joint[" << jointName << "\n";
    }

    jointElem = jointElem->GetNextElement("joint");
  }



  // Make sure at least one joint was created.
  if (this->joints.empty())
  {
    gzerr << "No joints specified in the harness plugin."
      << "The harness plugin will not run.\n";
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
        << "joint.\n";
    }
  }
  else
  {
    // Error reporting
    gzwarn << "A <detach> element is missing from the harness plugin. "
      << "The first joint will be used as the detach joint.\n";
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
          << "<winch> element of the harness plugin. The first joint will be "
          << "used as the winch.\n";
      }
    }
    else
    {
      // Error reporting
      gzwarn << "A <winch><joint>joint_name</joint></winch> element is "
        << "missing from the harness plugin. The first joint will be used "
        << "as the winch.\n";
    }

    // Load the PID controller
    if (winchElem->HasElement("pid"))
    {
      sdf::ElementPtr pidElem = winchElem->GetElement("pid");
      double pValue = pidElem->HasElement("p") ? pidElem->Get<double>("p") : 0;
      double iValue = pidElem->HasElement("i") ? pidElem->Get<double>("i") : 0;
      double dValue = pidElem->HasElement("d") ? pidElem->Get<double>("d") : 0;

      this->winchPID.Init(pValue, iValue, dValue);
    }
  }
  else
  {
    // Error reporting
    gzwarn << "A <winch> element is missing from the harness plugin. "
      << "The first joint will be used as the winch.\n";
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

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&HarnessPlugin::OnUpdate, this, std::placeholders::_1));
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

  // Calculate the velocity error
  double error = this->joints[this->winchIndex]->GetVelocity(0) -
    this->winchTargetVel;

  // Use the PID controller to compute the joint force
  double winchForce = this->winchPID.Update(error,
      _info.simTime - this->prevSimTime);

  // Apply the joint force
  this->joints[this->winchIndex]->SetForce(0, winchForce);

  this->prevSimTime = _info.simTime;
}

/////////////////////////////////////////////////
void HarnessPlugin::Detach()
{
  this->joints[this->detachIndex]->Detach();

  this->prevSimTime == common::Time::Zero;

  // We no longer need to update
  this->updateConnection.reset();
}

/////////////////////////////////////////////////
double HarnessPlugin::WinchVelocity() const
{
  return this->joints[this->winchIndex]->GetVelocity(0);
}

/////////////////////////////////////////////////
void HarnessPlugin::SetWinchVelocity(const float _value)
{
  this->winchTargetVel = _value;
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
    gzerr << "Inavlid velocity data[" << _msg->data() << "]\n";
  }
}

/////////////////////////////////////////////////
void HarnessPlugin::OnDetach(ConstGzStringPtr &_msg)
{
  if (_msg->data() == "true" || _msg->data() == "TRUE" ||
      _msg->data() == "True")
  {
    this->Detach();
  }
}
