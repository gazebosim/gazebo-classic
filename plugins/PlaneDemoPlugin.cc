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

#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "plugins/PlaneDemoPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(PlaneDemoPlugin)

struct JointControl
{
  public: std::string name;
  public: physics::JointPtr joint;
  public: double cmd;
  public: double incVal;
  public: int incKey;
  public: int decKey;
  public: common::PID pid;
};

struct EngineControl
{
  public: std::string name;
  public: physics::JointPtr joint;
  public: double maxTorque;
  public: int incKey;
  public: int decKey;
  public: double incVal;
  public: double torque;
};

struct ThrusterControl
{
  public: std::string name;
  public: physics::LinkPtr link;
  public: double maxTorque;
  public: int incKey;
  public: int decKey;
  public: math::Vector3 incVal;
  public: math::Vector3 force;
};

/// \brief Private data class
class gazebo::PlaneDemoPluginPrivate
{
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

  public: std::vector<EngineControl> engineControls;

  public: std::vector<ThrusterControl> thrusterControls;

  public: std::vector<JointControl> jointControls;

  public: common::Time lastUpdateTime;

  public: bool stop;
  public: void OnKeyHit();
  public: std::unique_ptr<std::thread> keyHitThread;
  public: std::mutex mutex;
};

/////////////////////////////////////////////////
PlaneDemoPlugin::PlaneDemoPlugin()
  : dataPtr(new PlaneDemoPluginPrivate)
{
}

/////////////////////////////////////////////////
PlaneDemoPlugin::~PlaneDemoPlugin()
{
  this->dataPtr->stop = true;
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

  gzerr << "model: " << this->dataPtr->model->GetName() << "\n";

  // get engine controls
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
          ec.torque = 0;
          this->dataPtr->engineControls.push_back(ec);
          gzdbg << "joint [" << jointName << "] controlled by keyboard"
                << " t[" << ec.maxTorque
                << "] +[" << ec.incKey
                << "] -[" << ec.decKey
                << "] d[" << ec.incVal
                << "].\n";
        }
      }
      // get next element
      enginePtr = enginePtr->GetNextElement("engine");
    }
  }

  // get thruster controls
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
            tc.incVal = thrusterPtr->Get<math::Vector3>("inc_val");
          tc.force = math::Vector3();
          this->dataPtr->thrusterControls.push_back(tc);
        }
      }
      // get next element
      thrusterPtr = thrusterPtr->GetNextElement("thruster");
    }
  }

  // get controls
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
  }
}

/////////////////////////////////////////////////
void PlaneDemoPlugin::Init()
{
  this->dataPtr->lastUpdateTime = this->dataPtr->world->GetSimTime();
  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&PlaneDemoPlugin::OnUpdate, this));
  this->dataPtr->keyHitThread.reset(
    new std::thread(std::bind(&PlaneDemoPluginPrivate::OnKeyHit,
        this->dataPtr.get())));
  this->dataPtr->stop = false;
}

/////////////////////////////////////////////////
void PlaneDemoPlugin::OnUpdate()
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

/////////////////////////////////////////////////
void PlaneDemoPluginPrivate::OnKeyHit()
{
  char ch='x';
  ch = getchar();
  while (!this->stop)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    printf("you hit");
    printf(" '%c'(%i)", isprint(ch)?ch:'?', static_cast<int>(ch));

    for (std::vector<EngineControl>::iterator ei = this->engineControls.begin();
      ei != this->engineControls.end(); ++ei)
    {
      if (static_cast<int>(ch) == ei->incKey)
      {
        // spin up motor
        ei->torque += ei->incVal;
        gzerr << "torque: " << ei->torque << "\n";
      }
      else if (static_cast<int>(ch) == ei->decKey)
      {
        ei->torque -= ei->incVal;
        gzerr << "torque: " << ei->torque << "\n";
      }
      else
      {
        // ungetc( ch, stdin );
        // gzerr << (int)ch << " : " << this->clIncKey << "\n";
      }
    }

    for (std::vector<ThrusterControl>::iterator
      ti = this->thrusterControls.begin();
      ti != this->thrusterControls.end(); ++ti)
    {
      if (static_cast<int>(ch) == ti->incKey)
      {
        // spin up motor
        ti->force += ti->incVal;
        gzerr << "force: " << ti->force << "\n";
      }
      else if (static_cast<int>(ch) == ti->decKey)
      {
        ti->force -= ti->incVal;
        gzerr << "force: " << ti->force << "\n";
      }
      else
      {
        // ungetc( ch, stdin );
        // gzerr << (int)ch << " : " << this->clIncKey << "\n";
      }
    }

    for (std::vector<JointControl>::iterator
      ji = this->jointControls.begin();
      ji != this->jointControls.end(); ++ji)
    {
      if (static_cast<int>(ch) == ji->incKey)
      {
        // spin up motor
        ji->cmd += ji->incVal;
        ji->pid.SetCmd(ji->cmd);
        gzerr << ji->joint->GetName()
              << " cur: " << ji->joint->GetAngle(0).Radian()
              << " cmd: " << ji->cmd << "\n";
      }
      else if (static_cast<int>(ch) == ji->decKey)
      {
        ji->cmd -= ji->incVal;
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
  }
}
