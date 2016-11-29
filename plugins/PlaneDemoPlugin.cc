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

#include <chrono>
#include <thread>
#include <ignition/math/Vector3.hh>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/PID.hh"
#include "gazebo/transport/transport.hh"
#include "plugins/PlaneDemoPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(PlaneDemoPlugin)

/// \brief Joint controller
struct JointControl
{
  /// \brief Pointer to the joint controlled by JointControl
  public: physics::JointPtr joint;

  /// \brief PID command
  public: double cmd;

  /// \brief Amount to increment the joint angle by on each update
  public: double incVal;

  /// \brief Max joint angle command
  public: double maxVal;

  /// \brief Min joint angle command
  public: double minVal;

  /// \brief Key for increasing the joint angle
  public: int incKey;

  /// \brief Key for decreasing the joint angle
  public: int decKey;

  /// \brief PID controller
  public: common::PID pid;
};

/// \brief Engine torque controller
struct EngineControl
{
  /// \brief Pointer to the joint controlled by EngineControl
  public: physics::JointPtr joint;

  /// \brief Max torque that can be applied to joint
  public: double maxTorque;

  /// \brief Key for increasing the engine torque
  public: int incKey;

  /// \brief Key for descreasing the engine torque
  public: int decKey;

  /// \brief Amount to increment the engine torque by on each update
  public: double incVal;

  /// \brief Max joint angle command
  public: double maxVal;

  /// \brief Min joint angle command
  public: double minVal;

  /// \brief Torque applied to engine joint
  public: double torque;
};

/// \brief Thruster force controller
struct ThrusterControl
{
  /// \brief Link controlled by ThrusterControl
  public: physics::LinkPtr link;

  /// \brief Key for increasing the thruster force
  public: int incKey;

  /// \brief Key for decreasing the thruster force
  public: int decKey;

  /// \brief Amount to increment the engine torque by on each update
  public: ignition::math::Vector3d incVal;

  /// \brief Max joint angle command
  public: ignition::math::Vector3d maxVal;

  /// \brief Min joint angle command
  public: ignition::math::Vector3d minVal;

  /// \brief Force applied to the thruster link
  public: ignition::math::Vector3d force;
};

/// \brief Private data class
class gazebo::PlaneDemoPluginPrivate
{
  /// \brief Callback when a keyboard message is received.
  /// \param[in] _msg Message containing the key press value.
  public: void OnKeyHit(ConstAnyPtr &_msg);

  /// \brief Connection to World Update events.
  public: event::ConnectionPtr updateConnection;

  /// \brief Pointer to world.
  public: physics::WorldPtr world;

  /// \brief Pointer to physics engine.
  public: physics::PhysicsEnginePtr physics;

  /// \brief Pointer to model containing plugin.
  public: physics::ModelPtr model;

  /// \brief SDF for this plugin;
  public: sdf::ElementPtr sdf;

  /// \brief A list of controls for the engine
  public: std::vector<EngineControl> engineControls;

  /// \brief A list of controls for the thruster
  public: std::vector<ThrusterControl> thrusterControls;

  /// \brief A list of controls for the joint
  public: std::vector<JointControl> jointControls;

  /// \brief Last update sim time
  public: common::Time lastUpdateTime;

  /// \brief Mutex to protect updates
  public: std::mutex mutex;

  /// \brief Pointer to a node for communication.
  public: transport::NodePtr gzNode;

  /// \brief State subscriber.
  public: transport::SubscriberPtr keyboardSub;
};

/////////////////////////////////////////////////
PlaneDemoPlugin::PlaneDemoPlugin()
  : dataPtr(new PlaneDemoPluginPrivate)
{
}

/////////////////////////////////////////////////
PlaneDemoPlugin::~PlaneDemoPlugin()
{
}

/////////////////////////////////////////////////
void PlaneDemoPlugin::Load(physics::ModelPtr _model,
                     sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "PlaneDemoPlugin _model pointer is NULL");
  this->dataPtr->model = _model;
  this->dataPtr->sdf = _sdf;

  this->dataPtr->world = this->dataPtr->model->GetWorld();
  GZ_ASSERT(this->dataPtr->world, "PlaneDemoPlugin world pointer is NULL");

  this->dataPtr->physics = this->dataPtr->world->GetPhysicsEngine();
  GZ_ASSERT(this->dataPtr->physics, "PlaneDemoPlugin physics pointer is NULL");

  GZ_ASSERT(_sdf, "PlaneDemoPlugin _sdf pointer is NULL");

  gzdbg << "using model: " << this->dataPtr->model->GetName() << "\n";

  // get engine controls
  gzdbg << "loading engines.\n";
  if (_sdf->HasElement("engine"))
  {
    sdf::ElementPtr enginePtr = _sdf->GetElement("engine");
    while (enginePtr)
    {
      if (enginePtr->HasElement("joint_name"))
      {
        std::string jointName = enginePtr->Get<std::string>("joint_name");
        physics::JointPtr joint = this->dataPtr->model->GetJoint(jointName);
        if (joint.get() != NULL)
        {
          EngineControl ec;
          // ec.name = enginePtr->GetAttribute("name")->GetAsString();
          ec.joint = joint;
          if (enginePtr->HasElement("max_torque"))
            ec.maxTorque = enginePtr->Get<double>("max_torque");
          if (enginePtr->HasElement("inc_key"))
            ec.incKey = enginePtr->Get<int>("inc_key");
          if (enginePtr->HasElement("dec_key"))
            ec.decKey = enginePtr->Get<int>("dec_key");
          if (enginePtr->HasElement("inc_val"))
            ec.incVal = enginePtr->Get<double>("inc_val");
          // initialize to joint limits
          ec.maxVal = joint->GetEffortLimit(0);
          ec.minVal = -joint->GetEffortLimit(0);
          // overwrite if user specified limits
          if (enginePtr->HasElement("max_val"))
            ec.maxVal = enginePtr->Get<double>("max_val");
          if (enginePtr->HasElement("min_val"))
            ec.minVal = enginePtr->Get<double>("min_val");
          ec.torque = 0;
          this->dataPtr->engineControls.push_back(ec);
          gzdbg << "joint [" << jointName << "] controlled by keyboard"
                << " t[" << ec.maxTorque
                << "] +[" << ec.incKey
                << "] -[" << ec.decKey
                << "] d[" << ec.incVal
                << "] max[" << ec.maxVal
                << "] min[" << ec.minVal
                << "].\n";
        }
      }
      // get next element
      enginePtr = enginePtr->GetNextElement("engine");
    }
  }

  // get thruster controls
  gzdbg << "loading thrusters.\n";
  if (_sdf->HasElement("thruster"))
  {
    sdf::ElementPtr thrusterPtr = _sdf->GetElement("thruster");
    while (thrusterPtr)
    {
      if (thrusterPtr->HasElement("link_name"))
      {
        std::string linkName = thrusterPtr->Get<std::string>("link_name");
        physics::LinkPtr link = this->dataPtr->model->GetLink(linkName);
        if (link.get() != NULL)
        {
          ThrusterControl tc;
          // tc.name = thrusterPtr->GetAttribute("name")->GetAsString();
          tc.link = link;
          if (thrusterPtr->HasElement("inc_key"))
            tc.incKey = thrusterPtr->Get<int>("inc_key");
          if (thrusterPtr->HasElement("dec_key"))
            tc.decKey = thrusterPtr->Get<int>("dec_key");
          if (thrusterPtr->HasElement("inc_val"))
            tc.incVal = thrusterPtr->Get<ignition::math::Vector3d>("inc_val");
          if (thrusterPtr->HasElement("max_val"))
            tc.maxVal = thrusterPtr->Get<ignition::math::Vector3d>("max_val");
          if (thrusterPtr->HasElement("min_val"))
            tc.minVal = thrusterPtr->Get<ignition::math::Vector3d>("min_val");
          tc.force = ignition::math::Vector3d::Zero;
          this->dataPtr->thrusterControls.push_back(tc);
        }
      }
      // get next element
      thrusterPtr = thrusterPtr->GetNextElement("thruster");
    }
  }

  // get controls
  gzdbg << "loading controls.\n";
  sdf::ElementPtr controlPtr = _sdf->GetElement("control");
  while (controlPtr)
  {
    if (controlPtr->HasElement("joint_name"))
    {
      std::string jointName = controlPtr->Get<std::string>("joint_name");
      physics::JointPtr joint = this->dataPtr->model->GetJoint(jointName);
      if (joint.get() != NULL)
      {
        JointControl jc;
        // jc.name = controlPtr->GetAttribute("name")->GetAsString();
        jc.joint = joint;
        if (controlPtr->HasElement("inc_key"))
          jc.incKey = controlPtr->Get<int>("inc_key");
        if (controlPtr->HasElement("dec_key"))
          jc.decKey = controlPtr->Get<int>("dec_key");
        if (controlPtr->HasElement("inc_val"))
          jc.incVal = controlPtr->Get<double>("inc_val");
        // initialize to joint limits
        jc.maxVal = joint->GetUpperLimit(0).Radian();
        jc.minVal = joint->GetLowerLimit(0).Radian();
        // overwrite if user specified limits
        if (controlPtr->HasElement("max_val"))
          jc.maxVal = controlPtr->Get<double>("max_val");
        if (controlPtr->HasElement("min_val"))
          jc.minVal = controlPtr->Get<double>("min_val");

        double p, i, d, iMax, iMin, cmdMax, cmdMin;

        if (controlPtr->HasElement("p"))
          p = controlPtr->Get<double>("p");
        else
          p = 0.0;

        if (controlPtr->HasElement("i"))
          i = controlPtr->Get<double>("i");
        else
          i = 0.0;

        if (controlPtr->HasElement("d"))
          d = controlPtr->Get<double>("d");
        else
          d = 0.0;

        if (controlPtr->HasElement("i_max"))
          iMax = controlPtr->Get<double>("i_max");
        else
          iMax = 0.0;

        if (controlPtr->HasElement("i_min"))
          iMin = controlPtr->Get<double>("i_min");
        else
          iMin = 0.0;

        if (controlPtr->HasElement("cmd_max"))
          cmdMax = controlPtr->Get<double>("cmd_max");
        else
          cmdMax = 1000.0;
        if (controlPtr->HasElement("cmd_min"))
          cmdMin = controlPtr->Get<double>("cmd_min");
        else
          cmdMin = -1000.0;
        jc.pid.Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
        jc.cmd = joint->GetAngle(0).Radian();
        jc.pid.SetCmd(jc.cmd);
        this->dataPtr->jointControls.push_back(jc);
      }
    }
    // get next element
    controlPtr = controlPtr->GetNextElement("control");
    // gzdbg << controlPtr << "\n";
  }

  // Initialize transport.
  this->dataPtr->gzNode = transport::NodePtr(new transport::Node());
  this->dataPtr->gzNode->Init();
  this->dataPtr->keyboardSub = this->dataPtr->gzNode->Subscribe<msgs::Any>(
    "~/keyboard/keypress", &PlaneDemoPluginPrivate::OnKeyHit,
    this->dataPtr.get());
  gzdbg << "Load done.\n";
}

/////////////////////////////////////////////////
void PlaneDemoPlugin::Init()
{
  this->dataPtr->lastUpdateTime = this->dataPtr->world->GetSimTime();
  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&PlaneDemoPlugin::OnUpdate, this));
  gzdbg << "Init done.\n";
}

/////////////////////////////////////////////////
void PlaneDemoPlugin::OnUpdate()
{
  // gzdbg << "executing OnUpdate.\n";
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    common::Time curTime = this->dataPtr->world->GetSimTime();
    for (std::vector<EngineControl>::iterator ei =
        this->dataPtr->engineControls.begin();
        ei != this->dataPtr->engineControls.end(); ++ei)
    {
      // spin up engine
      ei->joint->SetForce(0, ei->torque);
    }

    for (std::vector<ThrusterControl>::iterator
      ti = this->dataPtr->thrusterControls.begin();
      ti != this->dataPtr->thrusterControls.end(); ++ti)
    {
      // fire up thruster
      math::Pose pose = ti->link->GetWorldPose();
      ti->link->AddForce(pose.rot.RotateVector(ti->force));
    }

    for (std::vector<JointControl>::iterator ji =
        this->dataPtr->jointControls.begin();
        ji != this->dataPtr->jointControls.end(); ++ji)
    {
      // spin up joint control
      double pos = ji->joint->GetAngle(0).Radian();
      double error = pos - ji->cmd;
      double force = ji->pid.Update(error,
          curTime - this->dataPtr->lastUpdateTime);
      ji->joint->SetForce(0, force);
    }
    this->dataPtr->lastUpdateTime = curTime;
  }
}

/////////////////////////////////////////////////
void PlaneDemoPluginPrivate::OnKeyHit(ConstAnyPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  // gzdbg << "executing OnKeyHit.\n";
  char ch =_msg->int_value();
  gzdbg << "keyhit [" << ch
        << "] num [" << _msg->int_value() << "]\n";

  for (std::vector<EngineControl>::iterator ei = this->engineControls.begin();
    ei != this->engineControls.end(); ++ei)
  {
    if (static_cast<int>(ch) == ei->incKey)
    {
      // spin up motor
      ei->torque += ei->incVal;
    }
    else if (static_cast<int>(ch) == ei->decKey)
    {
      ei->torque -= ei->incVal;
    }
    else
    {
      // ungetc( ch, stdin );
      // gzerr << (int)ch << " : " << this->clIncKey << "\n";
    }
    ei->torque = math::clamp(ei->torque, ei->minVal, ei->maxVal);
    gzerr << "torque: " << ei->torque << "\n";
  }

  for (std::vector<ThrusterControl>::iterator
    ti = this->thrusterControls.begin();
    ti != this->thrusterControls.end(); ++ti)
  {
    if (static_cast<int>(ch) == ti->incKey)
    {
      // spin up motor
      ti->force += ti->incVal;
    }
    else if (static_cast<int>(ch) == ti->decKey)
    {
      ti->force -= ti->incVal;
    }
    else
    {
      // ungetc( ch, stdin );
      // gzerr << (int)ch << " : " << this->clIncKey << "\n";
    }
    ti->force.X() = math::clamp(ti->force.X(), ti->minVal.X(), ti->maxVal.X());
    ti->force.Y() = math::clamp(ti->force.Y(), ti->minVal.Y(), ti->maxVal.Y());
    ti->force.Z() = math::clamp(ti->force.Z(), ti->minVal.Z(), ti->maxVal.Z());
    gzerr << "force: " << ti->force << "\n";
  }

  for (std::vector<JointControl>::iterator
    ji = this->jointControls.begin();
    ji != this->jointControls.end(); ++ji)
  {
    if (static_cast<int>(ch) == ji->incKey)
    {
      // spin up motor
      ji->cmd += ji->incVal;
      ji->cmd = math::clamp(ji->cmd, ji->minVal, ji->maxVal);
      ji->pid.SetCmd(ji->cmd);
      gzerr << ji->joint->GetName()
            << " cur: " << ji->joint->GetAngle(0).Radian()
            << " cmd: " << ji->cmd << "\n";
    }
    else if (static_cast<int>(ch) == ji->decKey)
    {
      ji->cmd -= ji->incVal;
      ji->cmd = math::clamp(ji->cmd, ji->minVal, ji->maxVal);
      ji->pid.SetCmd(ji->cmd);
      gzerr << ji->joint->GetName()
            << " cur: " << ji->joint->GetAngle(0).Radian()
            << " cmd: " << ji->cmd << "\n";
    }
    else if (static_cast<int>(ch) == 99)  // 'c' resets all control surfaces
    {
      ji->cmd = 0;
      ji->pid.SetCmd(ji->cmd);
      gzerr << ji->joint->GetName()
            << " cur: " << ji->joint->GetAngle(0).Radian()
            << " cmd: " << ji->cmd << "\n";
    }
    else
    {
      // ungetc( ch, stdin );
      // gzerr << (int)ch << " : " << this->clIncKey << "\n";
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}
