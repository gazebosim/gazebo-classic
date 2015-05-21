/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include <algorithm>
#include <string>

#include <boost/thread.hpp>

#include "gazebo/math/Helpers.hh"

#include "gazebo/common/Assert.hh"
#include "gazebo/util/Joystick.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/util/Joystick.hh"
#include "plugins/DronePlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(DronePlugin)

/////////////////////////////////////////////////
DronePlugin::DronePlugin()
{
  this->joy = new util::Joystick();
  this->yawSpeed = 0;
  this->quit = false;
}

/////////////////////////////////////////////////
void DronePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model = _model;
  this->joy->Init(0);

  this->sdf = _sdf;

  this->baseJoint =
    this->model->GetJoint(this->sdf->Get<std::string>("base_joint"));
  if (!this->baseJoint)
  {
    gzerr << "<base_joint>" << this->sdf->Get<std::string>("base_joint")
          << "<base_joint> does not exist\n";
    return;
  }

  this->baseLink =
    this->model->GetLink(this->sdf->Get<std::string>("base_link"));
  if (!this->baseLink)
  {
    gzerr << "<base_link>" << this->sdf->Get<std::string>("base_link")
          << "<base_link> does not exist\n";
    return;
  }

  if (this->sdf->HasElement("base_pid_pos"))
  {
    sdf::ElementPtr basePidPos = this->sdf->GetElement("base_pid_pos");
    double pVal, iVal, dVal, cmdMaxVal, cmdMinVal;
    basePidPos->GetAttribute("p")->Get(pVal);
    basePidPos->GetAttribute("i")->Get(iVal);
    basePidPos->GetAttribute("d")->Get(dVal);
    basePidPos->GetAttribute("cmd_max")->Get(cmdMaxVal);
    basePidPos->GetAttribute("cmd_min")->Get(cmdMinVal);
    this->posPid.Init(pVal, iVal, dVal, 0, 0, cmdMaxVal, cmdMinVal);
  }
  else
  {
    gzwarn << "no <base_pid_pos> block, using defaults.\n";
    this->posPid.Init(10000, 0, 0, 0, 0, 10000, -10000);
  }

  if (this->sdf->HasElement("base_pid_rot"))
  {
    sdf::ElementPtr basePidRot = this->sdf->GetElement("base_pid_rot");
    double pVal, iVal, dVal, cmdMaxVal, cmdMinVal;
    basePidRot->GetAttribute("p")->Get(pVal);
    basePidRot->GetAttribute("i")->Get(iVal);
    basePidRot->GetAttribute("d")->Get(dVal);
    basePidRot->GetAttribute("cmd_max")->Get(cmdMaxVal);
    basePidRot->GetAttribute("cmd_min")->Get(cmdMinVal);
    this->rotPid.Init(pVal, iVal, dVal, 0, 0, cmdMaxVal, cmdMinVal);
  }
  else
  {
    gzwarn << "no <base_pid_rot> block, using defaults.\n";
    this->rotPid.Init(80000, 0, 0, 0, 0, 100000, -100000);
  }

  double baseJointImplicitDamping = 1.0;
  if (this->sdf->HasElement("damping"))
  {
    baseJointImplicitDamping = this->sdf->Get<double>("damping");
  }
  const double dampTol = 1.0e-6;
  if (baseJointImplicitDamping < dampTol)
  {
    gzwarn << "truncating arm base joint damping at " << dampTol << ".\n";
    baseJointImplicitDamping = dampTol;
  }
  // set damping
  this->baseJoint->SetParam("erp", 0, 0.0);
  this->baseJoint->SetParam("cfm", 0, 1.0/baseJointImplicitDamping);
  // same implicit damping for revolute joint stops
  this->baseJoint->SetParam("stop_erp", 0, 0.0);
  this->baseJoint->SetParam("stop_cfm", 0, 1.0/baseJointImplicitDamping);

  this->targetBaseLinkPose = this->baseLink->GetWorldPose();
  this->initPose = this->targetBaseLinkPose;

  this->baseLink->SetGravityMode(false);

  this->lastSimTime = this->model->GetWorld()->GetSimTime();

  /*physics::ShapePtr shape = this->model->GetWorld()->GetModel(
      "heightmap")->GetLink("link")->GetCollision("collision")->GetShape();
  this->heightmap = boost::dynamic_pointer_cast<physics::HeightmapShape>(shape);
  */

  this->testRay = boost::dynamic_pointer_cast<physics::RayShape>(
      this->model->GetWorld()->GetPhysicsEngine()->CreateShape(
        "ray", physics::CollisionPtr()));

  this->rotorJoints.push_back(this->model->GetJoint("iris::rotor_0_joint"));
  this->rotorJoints.push_back(this->model->GetJoint("iris::rotor_3_joint"));
  this->rotorJoints.push_back(this->model->GetJoint("iris::rotor_2_joint"));
  this->rotorJoints.push_back(this->model->GetJoint("iris::rotor_1_joint"));

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->worldControlPub =
      this->node->Advertise<msgs::WorldControl>(
          "~/world_control");

  this->timeLimit = common::Time(180);
  this->joyMutex = new boost::recursive_mutex();
  this->joyThread = new boost::thread(
      boost::bind(&DronePlugin::PollJoystick, this));

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&DronePlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
DronePlugin::~DronePlugin()
{
  this->quit = true;

  this->joyThread->join();
  delete this->joyThread;
  this->joyThread = NULL;

  delete this->joyMutex;
  this->joyMutex = NULL;

  delete this->joy;
  this->joy = NULL;
}

/////////////////////////////////////////////////
void DronePlugin::Reset()
{
  if (this->model)
    this->lastSimTime = this->model->GetWorld()->GetSimTime();

  this->targetBaseLinkPose = this->initPose;

  this->posPid.Reset();
  this->rotPid.Reset();

  this->wrench.force = math::Vector3::Zero;
  this->wrench.torque = math::Vector3::Zero;

  this->velocity = math::Vector3::Zero;
  this->yawSpeed = 0;

  if (this->baseLink)
    this->baseLink->SetGravityMode(false);

  this->timer.Stop();
  this->timer.Reset();
}

/////////////////////////////////////////////////
void DronePlugin::PollJoystick()
{
  while (!this->quit)
  {
    common::Time::MSleep(10);
    msgs::Joysticks msg;
    if (this->joy->Poll(msg))

    boost::recursive_mutex::scoped_lock lock(*this->joyMutex);
    this->joyMsg = msg;

    if (this->joyMsg.joy_size() > 0)
    {
      if (this->joyMsg.joy(0).button_size() > 0)
      {
        bool startButton = false;
        bool selectButton = false;
        for (int i = 0; i < this->joyMsg.joy(0).button_size(); ++i)
        {
          if (this->joyMsg.joy(0).button(i).index() == 3 &&
              this->joyMsg.joy(0).button(i).state() == 1)
            startButton = true;
          if (this->joyMsg.joy(0).button(i).index() == 0 &&
              this->joyMsg.joy(0).button(i).state() == 1)
            selectButton = true;
        }

        if (startButton || selectButton)
        {
          msgs::WorldControl worldControlMsg;
          if (selectButton)
            worldControlMsg.mutable_reset()->set_all(true);
          else if (startButton)
          {
            worldControlMsg.set_pause(!this->model->GetWorld()->IsPaused());
          }
          this->worldControlPub->Publish(worldControlMsg);
        }

      }
      // std::cout << this->joyMsg.DebugString();
    }
  }
}

/////////////////////////////////////////////////
void DronePlugin::OnUpdate()
{
  if (!this->timer.GetRunning())
  {
    math::Vector3 dist =
        this->baseLink->GetWorldPose().pos - this->initPose.pos;
    if (dist.GetLength() > 1.0)
    {
      this->timer.Start();
      std::cerr << "timer starts now!" << std::endl;
    }
  }
  else if (this->timer.GetElapsed() > this->timeLimit)
  {
    if (this->baseLink)
    {
      this->wrench.force = math::Vector3::Zero;
      this->wrench.torque = math::Vector3::Zero;
      this->baseLink->AddForceAtRelativePosition(this->wrench.force,
        math::Vector3(0, 0, 0.2));
      this->baseLink->AddTorque(this->wrench.torque);
      this->baseLink->SetGravityMode(true);
    }
    return;
  }


  common::Time curTime = this->model->GetWorld()->GetSimTime();
  common::Time dt = curTime - this->lastSimTime;
  this->lastSimTime = curTime;

  // msgs::Joysticks this->joyMsg;
  // if (this->joy->Poll(this->joyMsg))
  {
    boost::recursive_mutex::scoped_lock lock(*this->joyMutex);
    if (this->joyMsg.joy_size() > 0)
    {
      for (int i = 0; i < this->joyMsg.joy(0).analog_axis_size(); ++i)
      {
        if (this->joyMsg.joy(0).analog_axis(i).index() == 4)
        {
          // Forward motion (pitch)
          this->velocity.x =
              this->joyMsg.joy(0).analog_axis(i).value()/(-32768.0);
        }
        else if (this->joyMsg.joy(0).analog_axis(i).index() == 3)
        {
          // Lateral motion (roll)
          this->velocity.y =
              this->joyMsg.joy(0).analog_axis(i).value()/(-32768.0);
        }
        else if (this->joyMsg.joy(0).analog_axis(i).index() == 1)
        {
          // Up/down
          this->velocity.z =
              this->joyMsg.joy(0).analog_axis(i).value()/(-32768.0);
        }

        else if (this->joyMsg.joy(0).analog_axis(i).index() == 0)
        {
          // Yaw
          this->yawSpeed =
              this->joyMsg.joy(0).analog_axis(i).value()/(-32768.0);
        }
      }
    }

    // std::cout << "velocity[" << this->velocity << "]\n";
    // std::cout << "yawSpeed[" << this->yawSpeed << "]\n";
  }

  double direction = -1.0;
  for (physics::Joint_V::iterator j = this->rotorJoints.begin();
       j != this->rotorJoints.end(); ++j)
  {
    direction *= -1.0;
    if (*j)
      (*j)->SetVelocity(0, direction * (-100 * this->velocity.z + 60));
    else
      gzerr << "joint not found\n";
  }

  math::Pose baseLinkPose = this->baseLink->GetWorldPose();

  // rotate velocity to world frame
  const math::Vector3 maxVelocity(10.0, 10.0, 4.0);
  this->targetBaseLinkPose.pos += dt.Double() *
    baseLinkPose.rot.RotateVector(maxVelocity * this->velocity);

  double fakeTorqueScale = 2.0;
  math::Vector3 fakeRpyBodyFrame(-fakeTorqueScale * this->velocity.y,
   fakeTorqueScale * this->velocity.x, 0.0);

  math::Vector3 fakeRpyWorldFrame =
    baseLinkPose.rot.RotateVector(fakeRpyBodyFrame);

  // bounding box for target pose from body frame
  const math::Vector3 targetBB(5.0, 5.0, 2.0);
  this->targetBaseLinkPose.pos.x = math::clamp(targetBaseLinkPose.pos.x,
    baseLinkPose.pos.x-targetBB.x, baseLinkPose.pos.x+targetBB.x);
  this->targetBaseLinkPose.pos.y = math::clamp(targetBaseLinkPose.pos.y,
    baseLinkPose.pos.y-targetBB.y, baseLinkPose.pos.y+targetBB.y);
  this->targetBaseLinkPose.pos.z = math::clamp(targetBaseLinkPose.pos.z,
    baseLinkPose.pos.z-targetBB.z, baseLinkPose.pos.z+targetBB.z);

  math::Quaternion drpy;
  drpy.SetFromEuler(math::Vector3(0.0, 0.0, dt.Double() * yawSpeed));
  this->targetBaseLinkPose.rot = this->targetBaseLinkPose.rot * drpy;

  math::Vector3 errorPos = baseLinkPose.pos - this->targetBaseLinkPose.pos;

  math::Vector3 errorRot =
    (baseLinkPose.rot * this->targetBaseLinkPose.rot.GetInverse()).GetAsEuler();

  this->wrench.force.x = this->posPid.Update(errorPos.x, dt);
  this->wrench.force.y = this->posPid.Update(errorPos.y, dt);
  this->wrench.force.z = this->posPid.Update(errorPos.z, dt);
  this->baseLink->AddForceAtRelativePosition(this->wrench.force,
    math::Vector3(0, 0, -0.01));

  this->wrench.torque.x = this->rotPid.Update(errorRot.x, dt) +
    fakeRpyWorldFrame.x;
  this->wrench.torque.y = this->rotPid.Update(errorRot.y, dt) +
    fakeRpyWorldFrame.y;
  this->wrench.torque.z = this->rotPid.Update(errorRot.z, dt);

  this->baseLink->AddTorque(this->wrench.torque);
}
