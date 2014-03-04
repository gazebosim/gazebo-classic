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
#include "RobonautPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(RobonautPlugin)

/////////////////////////////////////////////////
RobonautPlugin::RobonautPlugin()
{
  this->activated = false;
  this->yaw = 0;
  gzerr << "Robonaut Plugin Constructor!" << std::endl;
}

/////////////////////////////////////////////////
RobonautPlugin::~RobonautPlugin()
{
}

/////////////////////////////////////////////////
void RobonautPlugin::OnHydra(ConstHydraPtr &_msg)
{
//  gzerr << " on hydra " << std::endl;
  if (_msg->right().button_center() &&_msg->left().button_center())
  {
    this->Restart();
    return;
  }

  boost::mutex::scoped_lock lock(this->msgMutex);
  this->hydraMsgs.push_back(_msg);
}

/////////////////////////////////////////////////
void RobonautPlugin::Restart()
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

  gzerr << "Robonaut Plugin Restart!" << std::endl;
  //this->world->Reset();
}

/////////////////////////////////////////////////
void RobonautPlugin::SetRightFingers(double _angle)
{
  this->jointController->SetPositionTarget(
      "r2::/r2/right_arm/hand/index/joint0", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/right_arm/hand/index/joint1", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/right_arm/hand/index/joint2", _angle);
  //this->jointController->SetPositionTarget(
  //    "r2::/r2/right_arm/hand/index/joint3", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/right_arm/hand/middle/joint0", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/right_arm/hand/middle/joint1", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/right_arm/hand/middle/joint2", _angle);
  //this->jointController->SetPositionTarget(
  //    "r2::/r2/right_arm/hand/middle/joint3", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/right_arm/hand/little/joint0", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/right_arm/hand/little/joint1", _angle);
  //this->jointController->SetPositionTarget(
  //    "r2::/r2/right_arm/hand/little/joint2", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/right_arm/hand/ring/joint0", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/right_arm/hand/ring/joint1", _angle);
  //this->jointController->SetPositionTarget(
  //    "r2::/r2/right_arm/hand/ring/joint2", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/right_arm/hand/thumb/joint0", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/right_arm/hand/thumb/joint1", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/right_arm/hand/thumb/joint2", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/right_arm/hand/thumb/joint3", _angle);
}

/////////////////////////////////////////////////
void RobonautPlugin::SetLeftFingers(double _angle)
{
  this->jointController->SetPositionTarget(
      "r2::/r2/left_arm/hand/index/joint0", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/left_arm/hand/index/joint1", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/left_arm/hand/index/joint2", _angle);
  //this->jointController->SetPositionTarget(
  //    "r2::/r2/left_arm/hand/index/joint3", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/left_arm/hand/middle/joint0", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/left_arm/hand/middle/joint1", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/left_arm/hand/middle/joint2", _angle);
  //this->jointController->SetPositionTarget(
  //    "r2::/r2/left_arm/hand/middle/joint3", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/left_arm/hand/little/joint0", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/left_arm/hand/little/joint1", _angle);
  //this->jointController->SetPositionTarget(
  //    "r2::/r2/left_arm/hand/little/joint2", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/left_arm/hand/ring/joint0", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/left_arm/hand/ring/joint1", _angle);
  //this->jointController->SetPositionTarget(
  //    "r2::/r2/left_arm/hand/ring/joint2", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/left_arm/hand/thumb/joint0", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/left_arm/hand/thumb/joint1", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/left_arm/hand/thumb/joint2", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/left_arm/hand/thumb/joint3", _angle);
}

/////////////////////////////////////////////////
void RobonautPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
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
      &RobonautPlugin::OnHydra, this);

  this->worldControlPub =
    this->node->Advertise<msgs::WorldControl>("~/world_control");

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

  this->pinJoint->Load(this->pinLink, this->model->GetLink("base"),
      math::Pose());
  this->pinJoint->SetUpperLimit(0,0);
  this->pinJoint->SetLowerLimit(0,0);
  this->pinJoint->Init();

  std::vector<std::string> rightHandJoints;
  rightHandJoints.push_back("/r2/right_arm/hand/index/joint0");
  rightHandJoints.push_back("/r2/right_arm/hand/index/joint1");
  rightHandJoints.push_back("/r2/right_arm/hand/index/joint2");
  rightHandJoints.push_back("/r2/right_arm/hand/index/joint3");
  rightHandJoints.push_back("/r2/right_arm/hand/middle/joint0");
  rightHandJoints.push_back("/r2/right_arm/hand/middle/joint1");
  rightHandJoints.push_back("/r2/right_arm/hand/middle/joint2");
  rightHandJoints.push_back("/r2/right_arm/hand/middle/joint3");
  rightHandJoints.push_back("/r2/right_arm/hand/little/joint0");
  rightHandJoints.push_back("/r2/right_arm/hand/little/joint1");
  rightHandJoints.push_back("/r2/right_arm/hand/little/joint2");
  rightHandJoints.push_back("/r2/right_arm/hand/ring/joint0");
  rightHandJoints.push_back("/r2/right_arm/hand/ring/joint1");
  rightHandJoints.push_back("/r2/right_arm/hand/ring/joint2");
  rightHandJoints.push_back("/r2/right_arm/hand/thumb/joint0");
  rightHandJoints.push_back("/r2/right_arm/hand/thumb/joint1");
  rightHandJoints.push_back("/r2/right_arm/hand/thumb/joint2");
  rightHandJoints.push_back("/r2/right_arm/hand/thumb/joint3");
  
  std::vector<std::string> leftHandJoints;
  leftHandJoints.push_back("/r2/left_arm/hand/index/joint0");
  leftHandJoints.push_back("/r2/left_arm/hand/index/joint1");
  leftHandJoints.push_back("/r2/left_arm/hand/index/joint2");
  leftHandJoints.push_back("/r2/left_arm/hand/index/joint3");
  leftHandJoints.push_back("/r2/left_arm/hand/middle/joint0");
  leftHandJoints.push_back("/r2/left_arm/hand/middle/joint1");
  leftHandJoints.push_back("/r2/left_arm/hand/middle/joint2");
  leftHandJoints.push_back("/r2/left_arm/hand/middle/joint3");
  leftHandJoints.push_back("/r2/left_arm/hand/little/joint0");
  leftHandJoints.push_back("/r2/left_arm/hand/little/joint1");
  leftHandJoints.push_back("/r2/left_arm/hand/little/joint2");
  leftHandJoints.push_back("/r2/left_arm/hand/ring/joint0");
  leftHandJoints.push_back("/r2/left_arm/hand/ring/joint1");
  leftHandJoints.push_back("/r2/left_arm/hand/ring/joint2");
  leftHandJoints.push_back("/r2/left_arm/hand/thumb/joint0");
  leftHandJoints.push_back("/r2/left_arm/hand/thumb/joint1");
  leftHandJoints.push_back("/r2/left_arm/hand/thumb/joint2");
  leftHandJoints.push_back("/r2/left_arm/hand/thumb/joint3");

  for (std::vector<std::string>::iterator iter = rightHandJoints.begin();
      iter != rightHandJoints.end(); ++iter)
  {
    this->jointController->SetPositionPID("r2::" + *iter,
        common::PID(3.0, 0.0, 0.1));

    this->jointController->SetPositionTarget("r2::" + *iter, 0);
  }

  for (std::vector<std::string>::iterator iter = leftHandJoints.begin();
      iter != leftHandJoints.end(); ++iter)
  {
    this->jointController->SetPositionPID("r2::" + *iter,
        common::PID(3.0, 0.0, 0.1));

    this->jointController->SetPositionTarget("r2::" + *iter, 0);
  }

  this->jointController->SetPositionPID(
      "r2::/r2/right_arm/hand/thumb/joint1",
      common::PID(0.5, 0.0, 0.1));
  this->jointController->SetPositionPID(
      "r2::/r2/left_arm/hand/thumb/joint1",
      common::PID(0.5, 0.0, 0.1));
   this->jointController->SetPositionPID(
      "r2::/r2/right_arm/hand/thumb/joint2",
      common::PID(0.5, 0.0, 0.1));
  this->jointController->SetPositionPID(
      "r2::/r2/left_arm/hand/thumb/joint2",
      common::PID(0.5, 0.0, 0.1));
 


  this->rightBumper = false;
  this->leftBumper = false;

  gzerr << "Robonaut Plugin Loaded!" << std::endl;

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&RobonautPlugin::Update, this, _1));
}

/////////////////////////////////////////////////
void RobonautPlugin::Update(const common::UpdateInfo & /*_info*/)
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



    rightAdjust = math::Pose(this->modelStartPose.GetInverse().rot *
        (rightPose.pos - this->resetPoseRight.pos) +
        this->basePoseRight.pos,
        this->modelStartPose.GetInverse().rot *
        rightPose.rot * this->resetPoseRight.rot.GetInverse() *
        this->basePoseRight.rot);

   /*std::cerr << " ----- " << std::endl;
   std::cerr << " right pose " << rightPose.pos << std::endl;
   std::cerr << " reset pose right " << resetPoseRight.pos << std::endl;
   std::cerr << " right - reset " << rightPose.pos - this->resetPoseRight.pos << std::endl;
   std::cerr << " base pose right " << basePoseRight.pos << std::endl;
   std::cerr << " model rot " << this->model->GetRelativePose().rot.GetAsEuler() << std::endl;
   std::cerr << " right adjust " << rightAdjust.pos << std::endl;*/
   


   //gzerr << " right adjust " << rightAdjust.pos << " vs " << this->model->GetRelativePose().rot*rightAdjust.pos << std::endl;

   // gzerr << " right adjust " << this->model->GetRelativePose().rot.GetAsEuler() << std::endl;

    leftAdjust = math::Pose(this->modelStartPose.GetInverse().rot *
        (leftPose.pos - this->resetPoseLeft.pos) +
        this->basePoseLeft.pos,
        this->modelStartPose.GetInverse().rot *
        leftPose.rot * this->resetPoseLeft.rot.GetInverse() *
        this->basePoseLeft.rot);

    //rightAdjust.pos = this->model->GetRelativePose().rot*rightAdjust.pos;
    //leftAdjust.pos = this->model->GetRelativePose().rot*leftAdjust.pos;

    this->SetRightFingers(msg->right().trigger()*1.5707);
    this->SetLeftFingers(msg->left().trigger()*1.5707);

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
    rpy.x = this->dollyStartPose.rot.x;
    rpy.y = this->dollyStartPose.rot.y;
    //std::cerr << " rpy x y " << rpy.x << " " << rpy.y << std::endl;

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
