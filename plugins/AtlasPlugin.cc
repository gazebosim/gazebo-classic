/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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

#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>

#include "gazebo/transport/transport.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/Joint.hh"
#include "AtlasPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(AtlasPlugin)

/////////////////////////////////////////////////
AtlasPlugin::AtlasPlugin()
{
  this->activated = false;
  this->yaw = 0;
}

/////////////////////////////////////////////////
AtlasPlugin::~AtlasPlugin()
{
}

/////////////////////////////////////////////////
void AtlasPlugin::OnHydra(ConstHydraPtr &_msg)
{
  if (_msg->right().button_center() &&_msg->left().button_center())
  {
    this->Restart();
    return;
  }

  math::Pose rightPose = msgs::Convert(_msg->right().pose());
  math::Pose leftPose = msgs::Convert(_msg->left().pose());

  if (!this->activated)
  {

    if (_msg->right().button_bumper() && _msg->left().button_bumper())
    {
      this->activated = true;
      this->resetPoseRight = rightPose;
      this->resetPoseLeft = leftPose;
    }
    else
      return;
  }

  math::Pose rightAdjust, leftAdjust;

  rightAdjust = math::Pose(rightPose.pos - this->resetPoseRight.pos +
      this->basePoseRight.pos,
      rightPose.rot * this->resetPoseRight.rot.GetInverse() *
      this->basePoseRight.rot);

  leftAdjust = math::Pose(leftPose.pos - this->resetPoseLeft.pos +
      this->basePoseLeft.pos,
      leftPose.rot * this->resetPoseLeft.rot.GetInverse() *
      this->basePoseLeft.rot);

  this->SetRightFingers(_msg->right().trigger()*1.5707);
  this->SetLeftFingers(_msg->left().trigger()*1.5707);


  double rx = _msg->right().joy_x() * .002;
  double ry = _msg->right().joy_y() * -.002;


  this->yaw += _msg->left().joy_y() * -.002;
  double dx = rx * cos(this->yaw) + ry*sin(this->yaw*-1);
  double dy = rx * sin(this->yaw) + ry*cos(this->yaw*-1);
  math::Pose dPose(dx, dy, 0, 0, 0, _msg->left().joy_y() * -.002);

  math::Vector3 rpy = this->dolly->GetWorldPose().rot.GetAsEuler();
  rpy.z = yaw;

  this->dollyPinJoint->Detach();
  math::Vector3 dollyPos = this->dolly->GetWorldPose().pos + dPose.pos;
  dollyPos.z = this->dollyStartPose.pos.z;
  this->dolly->SetWorldPose(math::Pose(dollyPos, math::Quaternion(rpy)));

  this->dollyPinJoint->Attach(physics::LinkPtr(), this->dolly->GetLink("link"));

  this->rightModel->SetRelativePose(rightAdjust);
  this->leftModel->SetRelativePose(leftAdjust);

  this->rightBumper = _msg->right().button_bumper();
  this->leftBumper = _msg->left().button_bumper();
}

void AtlasPlugin::Restart()
{
  this->rightModel->SetRelativePose(this->rightStartPose);
  this->leftModel->SetRelativePose(this->leftStartPose);
  this->basePoseRight = this->rightStartPose;
  this->basePoseLeft = this->leftStartPose;
  this->rightBumper = false;
  this->leftBumper = false;
  this->activated = false;
  this->yaw = 0;
  this->prevModelPose = this->modelStartPose;
  this->pelvisTarget = this->pelvisStartPose;

  this->dollyPinJoint->Detach();
  this->dolly->SetWorldPose(this->dollyStartPose);
  this->dollyPinJoint->Attach(physics::LinkPtr(), this->dolly->GetLink("link"));

  this->world->Reset();
}

/////////////////////////////////////////////////
void AtlasPlugin::SetRightFingers(double _angle)
{
  this->jointController->SetPositionTarget("atlas::right_f0_j1", _angle);
  this->jointController->SetPositionTarget("atlas::right_f0_j2", _angle);

  this->jointController->SetPositionTarget("atlas::right_f1_j1", _angle);
  this->jointController->SetPositionTarget("atlas::right_f1_j2", _angle);

  this->jointController->SetPositionTarget("atlas::right_f2_j1", _angle);
  this->jointController->SetPositionTarget("atlas::right_f2_j2", _angle);

  this->jointController->SetPositionTarget("atlas::right_f3_j1", _angle);
  this->jointController->SetPositionTarget("atlas::right_f3_j2", _angle);
}

/////////////////////////////////////////////////
void AtlasPlugin::SetLeftFingers(double _angle)
{
  this->jointController->SetPositionTarget("atlas::left_f0_j1", _angle);
  this->jointController->SetPositionTarget("atlas::left_f0_j2", _angle);

  this->jointController->SetPositionTarget("atlas::left_f1_j1", _angle);
  this->jointController->SetPositionTarget("atlas::left_f1_j2", _angle);

  this->jointController->SetPositionTarget("atlas::left_f2_j1", _angle);
  this->jointController->SetPositionTarget("atlas::left_f2_j2", _angle);

  this->jointController->SetPositionTarget("atlas::left_f3_j1", _angle);
  this->jointController->SetPositionTarget("atlas::left_f3_j2", _angle);
}

/////////////////////////////////////////////////
void AtlasPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Get the world name.
  this->model = _parent;
  this->world = this->model->GetWorld();

  this->rightModel = this->model->GetLink("right_arm_goal_link");
  this->leftModel = this->model->GetLink("left_arm_goal_link");

  if (!this->rightModel)
    gzerr << "Unable to get right arm goal link\n";

  if (!this->leftModel)
    gzerr << "Unable to get left arm goal link\n";

  math::Quaternion modelRot = this->model->GetWorldPose().rot;

  this->basePoseRight = this->rightModel->GetRelativePose();
  this->basePoseLeft = this->leftModel->GetRelativePose();

  this->rightStartPose = this->basePoseRight;
  this->leftStartPose = this->basePoseLeft;
  this->modelStartPose = this->model->GetWorldPose();

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->world->GetName());
  this->hydraSub = this->node->Subscribe("~/hydra",
      &AtlasPlugin::OnHydra, this);

  this->jointController = this->model->GetJointController();

  this->pinJoint = this->world->GetPhysicsEngine()->CreateJoint(
      "revolute", this->model);

  if (_sdf->HasElement("pin_link"))
  {
    std::string pinModelStr =_sdf->Get<std::string>("pin_model");
    std::string pinLinkStr =_sdf->Get<std::string>("pin_link");

    this->dolly = this->world->GetModel(pinModelStr);
    this->dollyStartPose = this->dolly->GetWorldPose();

    if (!this->dolly)
      gzerr << "Unable to get pin model[" << pinModelStr << "]\n";
    else
    {
      this->pinLink = this->dolly->GetLink(pinLinkStr);

      if (!this->pinLink)
        gzerr << "Unable to get pin link[" << pinLinkStr << "]\n";
    }

    this->dollyPinJoint = this->dolly->GetJoint("world_joint");
  }

  this->pinJoint->SetModel(this->model);

  this->pinJoint->Load(this->pinLink, this->model->GetLink("pelvis"),
      math::Pose());
  this->pinJoint->SetUpperLimit(0,0);
  this->pinJoint->SetLowerLimit(0,0);
  this->pinJoint->Init();

  std::vector<std::string> rightHandJoints;
  rightHandJoints.push_back("right_f0_j0");
  rightHandJoints.push_back("right_f0_j1");
  rightHandJoints.push_back("right_f0_j2");
  rightHandJoints.push_back("right_f1_j0");
  rightHandJoints.push_back("right_f1_j1");
  rightHandJoints.push_back("right_f1_j2");
  rightHandJoints.push_back("right_f2_j0");
  rightHandJoints.push_back("right_f2_j1");
  rightHandJoints.push_back("right_f2_j2");
  rightHandJoints.push_back("right_f3_j0");
  rightHandJoints.push_back("right_f3_j1");
  rightHandJoints.push_back("right_f3_j2");

  std::vector<std::string> leftHandJoints;
  leftHandJoints.push_back("left_f0_j0");
  leftHandJoints.push_back("left_f0_j1");
  leftHandJoints.push_back("left_f0_j2");
  leftHandJoints.push_back("left_f1_j0");
  leftHandJoints.push_back("left_f1_j1");
  leftHandJoints.push_back("left_f1_j2");
  leftHandJoints.push_back("left_f2_j0");
  leftHandJoints.push_back("left_f2_j1");
  leftHandJoints.push_back("left_f2_j2");
  leftHandJoints.push_back("left_f3_j0");
  leftHandJoints.push_back("left_f3_j1");
  leftHandJoints.push_back("left_f3_j2");

  for (std::vector<std::string>::iterator iter = rightHandJoints.begin();
      iter != rightHandJoints.end(); ++iter)
  {
    this->jointController->SetPositionPID("atlas::" + *iter,
        common::PID(80.5, 0.1, 2.1));

    this->jointController->SetPositionTarget("atlas::" + *iter, 0);
  }

  for (std::vector<std::string>::iterator iter = leftHandJoints.begin();
      iter != leftHandJoints.end(); ++iter)
  {
    this->jointController->SetPositionPID("atlas::" + *iter,
        common::PID(80.5, 0.1, 2.1));

    this->jointController->SetPositionTarget("atlas::" + *iter, 0);
  }

  this->rightBumper = false;
  this->leftBumper = false;

  /*this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&AtlasPlugin::Update, this, _1));

  this->model->GetLink("pelvis")->SetForce(math::Vector3(0, 0, 9.8));
  this->xPosPID.Init(1, 0, .10);
  this->yPosPID.Init(1, 0, .10);
  this->zPosPID.Init(10000, 0, 10);

  this->rollPID.Init(100, 0, 1);
  this->pitchPID.Init(100, 0, 1);
  this->yawPID.Init(50, 0, .1);

  this->pelvisTarget = this->model->GetLink("pelvis")->GetWorldPose();
  this->pelvisStartPose = this->pelvisTarget;

  this->prevModelPose = this->model->GetWorldPose();
  */
}

/////////////////////////////////////////////////
void AtlasPlugin::Update(const common::UpdateInfo &/*_info*/)
{
  math::Pose currentModelPose = this->model->GetWorldPose();

  math::Pose pelvisCurrent = this->model->GetLink("pelvis")->GetWorldPose();
  math::Vector3 rpy = pelvisCurrent.rot.GetAsEuler();
  math::Vector3 targetRPY = this->pelvisTarget.rot.GetAsEuler();

  math::Vector3 err = pelvisCurrent.pos - this->pelvisTarget.pos;
  math::Vector3 rpyErr = math::Vector3(rpy.x, rpy.y, rpy.z - targetRPY.z);
  math::Vector3 force, torque;

  force.x = this->xPosPID.Update(err.x, common::Time(0, 100000));
  force.y = this->yPosPID.Update(err.y, common::Time(0, 100000));
  force.z = this->zPosPID.Update(err.z, common::Time(0, 100000));

  torque.x = this->rollPID.Update(rpyErr.x, common::Time(0, 100000));
  torque.y = this->pitchPID.Update(rpyErr.y, common::Time(0, 100000));
  torque.z = this->yawPID.Update(rpyErr.z, common::Time(0, 100000));

  /*std::cout << "Err[" << rpyErr << "] Torque[" << torque << "]\n";

  this->model->GetLink("pelvis")->SetForce(force);
  this->model->GetLink("pelvis")->SetTorque(torque);
  */

  this->prevModelPose = this->model->GetWorldPose();
}
