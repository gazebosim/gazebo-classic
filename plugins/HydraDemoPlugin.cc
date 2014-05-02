/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#include "gazebo/transport/transport.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/Joint.hh"
#include "HydraDemoPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(HydraDemoPlugin)

/////////////////////////////////////////////////
HydraDemoPlugin::HydraDemoPlugin()
{
  this->activated = false;
  this->yaw = 0;
}

/////////////////////////////////////////////////
HydraDemoPlugin::~HydraDemoPlugin()
{
}

/////////////////////////////////////////////////
void HydraDemoPlugin::OnHydra(ConstHydraPtr &_msg)
{
  if (_msg->right().button_center() &&_msg->left().button_center())
  {
    this->Restart();
    return;
  }

  boost::mutex::scoped_lock lock(this->msgMutex);
  this->hydraMsgs.push_back(_msg);
}

/////////////////////////////////////////////////
void HydraDemoPlugin::Restart()
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

  msgs::WorldControl msg;
  msg.mutable_reset()->set_all(true);
  this->worldControlPub->Publish(msg);
}

/////////////////////////////////////////////////
/*void HydraDemoPlugin::SetRightFingers(double _angle)
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
void HydraDemoPlugin::SetLeftFingers(double _angle)
{
  this->jointController->SetPositionTarget("atlas::left_f0_j1", _angle);
  this->jointController->SetPositionTarget("atlas::left_f0_j2", _angle);

  this->jointController->SetPositionTarget("atlas::left_f1_j1", _angle);
  this->jointController->SetPositionTarget("atlas::left_f1_j2", _angle);

  this->jointController->SetPositionTarget("atlas::left_f2_j1", _angle);
  this->jointController->SetPositionTarget("atlas::left_f2_j2", _angle);

  this->jointController->SetPositionTarget("atlas::left_f3_j1", _angle);
  this->jointController->SetPositionTarget("atlas::left_f3_j2", _angle);
}*/

/////////////////////////////////////////////////
void HydraDemoPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
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
      &HydraDemoPlugin::OnHydra, this);

  this->worldControlPub =
    this->node->Advertise<msgs::WorldControl>("~/world_control");

  //this->jointController = this->model->GetJointController();

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

  /*std::vector<std::string> rightHandJoints;
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
  }*/

  this->rightBumper = false;
  this->leftBumper = false;

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&HydraDemoPlugin::Update, this, _1));
}

/////////////////////////////////////////////////
void HydraDemoPlugin::Update(const common::UpdateInfo & /*_info*/)
{
  boost::mutex::scoped_lock lock(this->msgMutex);

  if (this->hydraMsgs.empty())
    return;

  if (!this->activated)
  {
    for (std::list<boost::shared_ptr<msgs::Hydra const> >::iterator iter =
        this->hydraMsgs.begin(); iter != this->hydraMsgs.end(); ++iter)
    {
      if ((*iter)->right().button_bumper() && (*iter)->left().button_bumper())
      {
        this->activated = true;
        this->resetPoseRight = msgs::Convert((*iter)->right().pose());
        this->resetPoseLeft = msgs::Convert((*iter)->left().pose());
        break;
      }
    }
  }

  if (this->activated)
  {
    boost::shared_ptr<msgs::Hydra const> msg = this->hydraMsgs.back();

    math::Pose rightPose;
    math::Pose leftPose;

    math::Pose rightAdjust, leftAdjust;

    rightPose = msgs::Convert(msg->right().pose());
    leftPose = msgs::Convert(msg->left().pose());

    /*rightAdjust = math::Pose(rightPose.pos - this->resetPoseRight.pos +
        this->basePoseRight.pos,
        rightPose.rot * this->resetPoseRight.rot.GetInverse() *
        this->basePoseRight.rot);

    leftAdjust = math::Pose(leftPose.pos - this->resetPoseLeft.pos +
        this->basePoseLeft.pos,
        leftPose.rot * this->resetPoseLeft.rot.GetInverse() *
        this->basePoseLeft.rot);*/

    /*this->SetRightFingers(msg->right().trigger() * 1.5707);
    this->SetLeftFingers(msg->left().trigger() * 1.5707);*/

    common::Time curTime = common::Time::GetWallTime();
    double dt = (curTime - this->prevTime).Double();

    double rx = msg->right().joy_x() * 0.5 * dt;
    double ry = msg->right().joy_y() * -0.5 * dt;

    this->yaw += msg->left().joy_y() * -.5 * dt;
    double dx = rx * cos(this->yaw) + ry*sin(this->yaw*-1);
    double dy = rx * sin(this->yaw) + ry*cos(this->yaw*-1);
    math::Pose dPose(dx, dy, 0, 0, 0, msg->left().joy_y() * -.5 * dt);

    math::Vector3 rpy = this->dolly->GetWorldPose().rot.GetAsEuler();
    rpy.z = yaw;

    this->dollyPinJoint->Detach();
    math::Vector3 dollyPos = this->dolly->GetWorldPose().pos + dPose.pos;
    dollyPos.z = this->dollyStartPose.pos.z;
    this->dolly->SetWorldPose(math::Pose(dollyPos, math::Quaternion(rpy)));

    this->dollyPinJoint->Attach(physics::LinkPtr(),
        this->dolly->GetLink("link"));

    this->rightModel->SetRelativePose(rightAdjust);
    this->leftModel->SetRelativePose(leftAdjust);

    this->rightBumper = msg->right().button_bumper();
    this->leftBumper = msg->left().button_bumper();

    this->prevTime = curTime;
  }

  this->hydraMsgs.clear();
}
