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


#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/OculusCamera.hh"

#include "gazebo/math/Helpers.hh"

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Assert.hh"
#include "gazebo/util/Joystick.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "plugins/DronePlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(DronePlugin)

/////////////////////////////////////////////////
DronePlugin::DronePlugin()
{
  this->joy = new util::Joystick();
  this->yawSpeed = 0;
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
    this->rotPid.Init(10000, 0, 0, 0, 0, 10000, -10000);
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

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&DronePlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void DronePlugin::OnUpdate()
{
  common::Time curTime = this->model->GetWorld()->GetSimTime();
  common::Time dt = curTime - this->lastSimTime;
  this->lastSimTime = curTime;

  msgs::Joysticks msg;
  if (this->joy->Poll(msg))
  {
    if (msg.joy_size() > 0)
    {
      for (int i = 0; i < msg.joy(0).analog_axis_size(); ++i)
      {
        if (msg.joy(0).analog_axis(i).index() == 3)
        {
          // Forward motion (pitch)
          this->velocity.x = msg.joy(0).analog_axis(i).value()/(-32768.0);
        }
        else if (msg.joy(0).analog_axis(i).index() == 2)
        {
          // Lateral motion (roll)
          this->velocity.y = msg.joy(0).analog_axis(i).value()/(-32768.0);
        }
        else if (msg.joy(0).analog_axis(i).index() == 1)
        {
          // Up/down
          this->velocity.z = msg.joy(0).analog_axis(i).value()/(-32768.0);
        }

        else if (msg.joy(0).analog_axis(i).index() == 0)
        {
          // Yaw
          this->yawSpeed = msg.joy(0).analog_axis(i).value()/(-32768.0);
        }

      }
      // std::cout << msg.DebugString();
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
      (*j)->SetVelocity(0, direction * (100 * this->velocity.z + 60));
    else
      gzerr << "joint not found\n";
  }

  math::Pose baseLinkPose = this->baseLink->GetWorldPose();

  // rotate velocity to world frame
  const math::Vector3 maxVelocity(10.0, 10.0, 4.0);
  this->targetBaseLinkPose.pos += dt.Double() *
    baseLinkPose.rot.RotateVector(maxVelocity * this->velocity);

  double fakeTorqueScale = 5.0;
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

  // zero out pitch and roll
  // math::Vector3 rpy = this->targetBaseLinkPose.rot.GetAsEuler();
  // this->targetBaseLinkPose.rot.SetFromEuler(rpy);

  math::Quaternion drpy;
  drpy.SetFromEuler(math::Vector3(0.0, 0.0, dt.Double() * yawSpeed));
  this->targetBaseLinkPose.rot = this->targetBaseLinkPose.rot * drpy;

  math::Vector3 errorPos = baseLinkPose.pos - this->targetBaseLinkPose.pos;

  // std::cout << "curret[" << baseLinkPose << "]\n";
  // std::cout << "target[" << this->targetBaseLinkPose << "]\n";
  math::Vector3 errorRot =
    (baseLinkPose.rot * this->targetBaseLinkPose.rot.GetInverse()).GetAsEuler();

  this->wrench.force.x = this->posPid.Update(errorPos.x, dt);
  this->wrench.force.y = this->posPid.Update(errorPos.y, dt);
  this->wrench.force.z = this->posPid.Update(errorPos.z, dt);
  this->baseLink->AddForceAtRelativePosition(this->wrench.force,
    math::Vector3(0, 0, 0.2));

  this->wrench.torque.x = this->rotPid.Update(errorRot.x, dt) + fakeRpyWorldFrame.x;
  this->wrench.torque.y = this->rotPid.Update(errorRot.y, dt) + fakeRpyWorldFrame.y;
  this->wrench.torque.z = this->rotPid.Update(errorRot.z, dt);

  // math::Vector3 curRpy = baseLinkPose.rot.GetAsEuler();
  // this->wrench.torque.x = this->rotPid.Update(curRpy.x, dt);
  // this->wrench.torque.y = this->rotPid.Update(curRpy.y, dt);

  // double izzDt = 20.0 * 0.004/0.001;
  // this->wrench.torque.z = izzDt * this->yawSpeed;

  this->baseLink->AddTorque(this->wrench.torque);

/*
  math::Pose pose = this->model->GetWorldPose();
  pose.pos = pose.rot.RotateVector(
      this->velocity * math::Vector3(0.02,0.02,0.01)) + pose.pos;

  math::Vector3 rpy = pose.rot.GetAsEuler();
  rpy.z += this->yawSpeed * 0.001;
  pose.rot.SetFromEuler(rpy);

  this->testRay->SetPoints(pose.pos - math::Vector3(0,0,0.5),
                           pose.pos - math::Vector3(0,0,100));
  std::string entityName;
  double dist;
  this->testRay->GetIntersection(dist, entityName);

  double height = pose.pos.z - dist;
  // Limit the maximum height
  pose.pos.z = math::clamp(pose.pos.z, height+0.2, height+40.0);

  this->model->SetWorldPose(pose);
*/
}
