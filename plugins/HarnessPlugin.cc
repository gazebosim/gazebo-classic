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
#include <mutex>
#include <boost/thread/recursive_mutex.hpp>
#include <sdf/sdf.hh>

#include "gazebo/common/PID.hh"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Subscriber.hh"

#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/World.hh"

#include "plugins/HarnessPlugin.hh"

namespace gazebo
{
  /// \brief Private data for the HarnessPlugin class
  class HarnessPluginPrivate
  {
    /// \brief sdf pointer to the <plugin> block
    public: sdf::ElementPtr sdf;

    /// \brief model pointer
    public: physics::ModelPtr model;

    /// \brief Vector of joints
    public: std::vector<physics::JointPtr> joints;

    /// \brief Mutex guarding access to joints vector and index variables.
    public: std::recursive_mutex jointsMutex;

    /// \brief Index into the joints vector that specifies the winch joint.
    public: int winchIndex = -1;

    /// \brief Index into the joints vector that specifies the joint to detach.
    public: int detachIndex = -1;

    /// \brief Name of child link of detach joint.
    /// This is used when re-attaching.
    public: std::string detachLinkName;

    /// \brief Position PID controller for the winch
    public: common::PID winchPosPID;

    /// \brief Velocity PID controller for the winch
    public: common::PID winchVelPID;

    /// \brief Target winch position
    public: float winchTargetPos = 0.0;

    /// \brief Target winch velocity
    public: float winchTargetVel = 0.0;

    /// \brief Previous simulation time
    public: common::Time prevSimTime = common::Time::Zero;

    /// \brief Communication node
    /// \todo: Transition to ignition-transport in gazebo8
    public: transport::NodePtr node;

    /// \brief Velocity control subscriber
    /// \todo: Transition to ignition-transport in gazebo8
    public: transport::SubscriberPtr velocitySub;

    /// \brief Attach control subscriber
    /// \todo: Transition to ignition-transport in gazebo8
    public: transport::SubscriberPtr attachSub;

    /// \brief Detach control subscriber
    /// \todo: Transition to ignition-transport in gazebo8
    public: transport::SubscriberPtr detachSub;

    /// \brief Connection to World Update events.
    public: event::ConnectionPtr updateConnection;
  };
}

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(HarnessPlugin)

/////////////////////////////////////////////////
HarnessPlugin::HarnessPlugin()
  : dataPtr(new HarnessPluginPrivate)
{
}

/////////////////////////////////////////////////
HarnessPlugin::~HarnessPlugin()
{
  // clear subscribers and transport
  this->dataPtr->updateConnection.reset();
  this->dataPtr->detachSub.reset();
  this->dataPtr->attachSub.reset();
  this->dataPtr->velocitySub.reset();
  if (this->dataPtr->node)
    this->dataPtr->node->Fini();
  this->dataPtr->node.reset();
}

/////////////////////////////////////////////////
void HarnessPlugin::Load(physics::ModelPtr _model,
                         sdf::ElementPtr _sdf)
{
  // store pointers
  this->dataPtr->model = _model;
  this->dataPtr->sdf = _sdf;

  // Load all the harness joints
  this->Attach();

  // Get the winch
  if (_sdf->HasElement("winch"))
  {
    sdf::ElementPtr winchElem = _sdf->GetElement("winch");

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

      this->dataPtr->winchPosPID.Init(pValue, iValue, dValue,
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

      this->dataPtr->winchVelPID.Init(pValue, iValue, dValue,
        iMax, iMin, cmdMax, cmdMin);
    }
  }
}

/////////////////////////////////////////////////
void HarnessPlugin::Init()
{
  // Get a pointer to the world
  physics::WorldPtr world = this->dataPtr->model->GetWorld();

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init(world->Name());

  this->dataPtr->velocitySub = this->dataPtr->node->Subscribe(
      "~/" + this->dataPtr->model->GetName() + "/harness/velocity",
      &HarnessPlugin::OnVelocity, this);

  this->dataPtr->attachSub = this->dataPtr->node->Subscribe(
      "~/" + this->dataPtr->model->GetName() + "/harness/attach",
      &HarnessPlugin::OnAttach, this);

  this->dataPtr->detachSub = this->dataPtr->node->Subscribe(
      "~/" + this->dataPtr->model->GetName() + "/harness/detach",
      &HarnessPlugin::OnDetach, this);

  if (!this->dataPtr->joints.empty())
  {
    this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&HarnessPlugin::OnUpdate, this, std::placeholders::_1));
  }
}

/////////////////////////////////////////////////
void HarnessPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  // Bootstrap the time.
  if (this->dataPtr->prevSimTime == common::Time::Zero)
  {
    this->dataPtr->prevSimTime = _info.simTime;
    return;
  }
  common::Time dt = _info.simTime - this->dataPtr->prevSimTime;

  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->jointsMutex);
  const int joints_size = static_cast<int>(this->dataPtr->joints.size());
  const int winchIndex = this->dataPtr->winchIndex;
  if (winchIndex < 0 ||
      winchIndex >= joints_size)
  {
    if (this->dataPtr->detachIndex >= 0 &&
        this->dataPtr->detachIndex < joints_size)
    {
      gzmsg << "Detaching harness joint" << std::endl;
      this->Detach();
      return;
    }
    gzerr << "No known winch joint to control" << std::endl;
    return;
  }

  double pError = 0;
  if (ignition::math::equal(this->dataPtr->winchTargetVel, 0.0f))
  {
    // Calculate the position error if vel target is 0
    pError = this->dataPtr->joints[winchIndex]->Position(0) -
      this->dataPtr->winchTargetPos;
  }

  // Calculate the velocity error
  double vError = this->dataPtr->joints[winchIndex]->GetVelocity(0) -
    this->dataPtr->winchTargetVel;

  // Use the PID controller to compute the joint force
  double winchPosForce = this->dataPtr->winchPosPID.Update(pError, dt);
  double winchVelForce = this->dataPtr->winchVelPID.Update(vError, dt);

  // Truncate winchForce so it doesn't push the robot downwards
  // although this can also be accomplished by cmd_min and cmd_max.
  winchVelForce = winchVelForce > 0? winchVelForce : 0.0;

  // Apply the joint force
  this->dataPtr->joints[winchIndex]->SetForce(0, winchVelForce + winchPosForce);

  this->dataPtr->prevSimTime = _info.simTime;
}

/////////////////////////////////////////////////
void HarnessPlugin::Attach()
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->jointsMutex);
  // Load all the harness joints
  sdf::ElementPtr jointElem = this->dataPtr->sdf->GetElement("joint");
  while (jointElem)
  {
    std::string jointName = jointElem->Get<std::string>("name");
    try
    {
      auto joint = this->dataPtr->model->CreateJoint(jointElem);
      this->dataPtr->joints.push_back(joint);
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
  if (this->dataPtr->joints.empty())
  {
    gzerr << "No joints specified in the harness plugin."
          << "The harness plugin will not run."
          << std::endl;
    return;
  }

  // Get the detach joint
  if (this->dataPtr->sdf->HasElement("detach"))
  {
    std::string jointName = this->dataPtr->sdf->Get<std::string>("detach");
    this->dataPtr->detachIndex = this->JointIndex(jointName);

    // Error reporting
    if (this->dataPtr->detachIndex < 0)
    {
      this->dataPtr->detachIndex = 0;
      gzwarn << "Invalid <detach> joint name[" << jointName << "] in the "
             << "harness plugin. The first joint will be used as the detach "
             << "joint."
             << std::endl;
    }

    // Get name of child link of detach joint for when we re-attach.
    auto child = this->dataPtr->joints[this->dataPtr->detachIndex]->GetChild();
    if (child)
    {
      this->dataPtr->detachLinkName = child->GetName();
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
  if (this->dataPtr->sdf->HasElement("winch"))
  {
    sdf::ElementPtr winchElem = this->dataPtr->sdf->GetElement("winch");

    // Find the winch joint
    if (winchElem->HasElement("joint"))
    {
      std::string winchJointName = winchElem->Get<std::string>("joint");
      this->dataPtr->winchIndex = this->JointIndex(winchJointName);

      // Error reporting
      if (this->dataPtr->winchIndex < 0)
      {
        this->dataPtr->winchIndex = 0;
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
  }
  else
  {
    // Error reporting
    gzwarn << "A <winch> element is missing from the harness plugin. "
           << "The first joint will be used as the winch."
           << std::endl;
  }

  // Init the joints
  for (auto &joint : this->dataPtr->joints)
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
}

/////////////////////////////////////////////////
void HarnessPlugin::Attach(const ignition::math::Pose3d &_pose)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->jointsMutex);
  if (this->dataPtr->detachIndex >= 0 || this->dataPtr->winchIndex >= 0)
  {
    gzerr << "Winch or detach joints already exist, unable to attach new joint"
          << std::endl;
    return;
  }

  ignition::math::Pose3d detachLinkOffset;
  auto link = this->dataPtr->model->GetLink(this->dataPtr->detachLinkName);
  if (link)
  {
    detachLinkOffset = link->WorldPose() - this->dataPtr->model->WorldPose();
  }
  else
  {
    gzerr << "Unable to determine link to set pose, default to canonical link"
          << std::endl;
  }
  this->dataPtr->model->SetWorldPose(-detachLinkOffset + _pose);
  this->Attach();
  this->dataPtr->winchTargetPos = 0;
  this->dataPtr->winchTargetVel = 0;
  if (!this->dataPtr->joints.empty())
  {
    this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&HarnessPlugin::OnUpdate, this, std::placeholders::_1));
  }
}

/////////////////////////////////////////////////
void HarnessPlugin::Detach()
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->jointsMutex);
  const int joints_size = static_cast<int>(this->dataPtr->joints.size());
  if (this->dataPtr->detachIndex < 0 ||
      this->dataPtr->detachIndex >= joints_size)
  {
    gzerr << "No known joint to detach" << std::endl;
    return;
  }
  const auto detachName =
      this->dataPtr->joints[this->dataPtr->detachIndex]->GetName();
  physics::BasePtr parent =
      this->dataPtr->joints[this->dataPtr->detachIndex]->Base::GetParent();

  auto model = boost::dynamic_pointer_cast<physics::Model>(parent);
  if (!model)
  {
    gzerr << "Can't get valid model pointer" << std::endl;
    return;
  }

  // We no longer need to update
  this->dataPtr->updateConnection.reset();

  (this->dataPtr->joints[this->dataPtr->detachIndex]).reset();
  model->RemoveJoint(detachName);
  this->dataPtr->detachIndex = -1;
  this->dataPtr->winchIndex = -1;
  this->dataPtr->joints.clear();

  this->dataPtr->prevSimTime == common::Time::Zero;
}

/////////////////////////////////////////////////
double HarnessPlugin::WinchVelocity() const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->jointsMutex);
  const int winchIndex = this->dataPtr->winchIndex;
  if (winchIndex < 0 ||
      winchIndex >= static_cast<int>(this->dataPtr->joints.size()))
  {
    gzerr << "No known winch joint to get velocity" << std::endl;
    return 0;
  }
  return this->dataPtr->joints[winchIndex]->GetVelocity(0);
}

/////////////////////////////////////////////////
void HarnessPlugin::SetWinchVelocity(const float _value)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->jointsMutex);
  const int winchIndex = this->dataPtr->winchIndex;
  if (winchIndex < 0 ||
      winchIndex >= static_cast<int>(this->dataPtr->joints.size()))
  {
    gzerr << "No known winch joint to set velocity" << std::endl;
    return;
  }

  this->dataPtr->winchTargetVel = _value;
  if (ignition::math::equal(_value, 0.0f))
  {
    // if zero velocity is commanded, hold position
    this->dataPtr->winchTargetPos =
        this->dataPtr->joints[winchIndex]->Position(0);
    this->dataPtr->winchPosPID.Reset();
  }
}

/////////////////////////////////////////////////
int HarnessPlugin::JointIndex(const std::string &_name) const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->jointsMutex);
  // Find the given joint in our list of joints
  for (size_t i = 0; i < this->dataPtr->joints.size(); ++i)
  {
    if (this->dataPtr->joints[i]->GetName() == _name)
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
void HarnessPlugin::OnAttach(ConstPosePtr &_msg)
{
  // Block physics updates during harness attachment
  boost::recursive_mutex::scoped_lock lock(
      *this->dataPtr->model->GetWorld()->Physics()->GetPhysicsUpdateMutex());
  this->Attach(msgs::ConvertIgn(*_msg));
}

/////////////////////////////////////////////////
void HarnessPlugin::OnDetach(ConstGzStringPtr &_msg)
{
  if (_msg->data() == "true" ||
      _msg->data() == "TRUE" ||
      _msg->data() == "True")
  {
    std::lock_guard<std::recursive_mutex> lock(this->dataPtr->jointsMutex);
    this->dataPtr->winchIndex = -1;
  }
}
