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

  if (!this->activated)
  {
    if (_msg->right().button_bumper() && _msg->left().button_bumper())
      this->activated = true;
    else
      return;
  }

  math::Pose rightAdjust, leftAdjust;

  rightAdjust = this->rightModel->GetWorldPose();
  leftAdjust = this->leftModel->GetWorldPose();

  //if (_msg->right().button_bumper())
  {
    math::Pose rightPose = msgs::Convert(_msg->right().pose());

    if (this->rightBumper != _msg->right().button_bumper())
    {
      this->resetPoseRight = rightPose;
    }

    rightAdjust = math::Pose(rightPose.pos - this->resetPoseRight.pos +
        this->basePoseRight.pos,
        rightPose.rot * this->resetPoseRight.rot.GetInverse() *
        this->basePoseRight.rot);
  }
  /*else
  {
    this->basePoseRight = rightAdjust;
  }

  if (_msg->left().button_bumper())
  */
  {
    math::Pose leftPose = msgs::Convert(_msg->left().pose());

    if (this->leftBumper != _msg->left().button_bumper())
    {
      this->resetPoseLeft = leftPose;
    }

    leftAdjust = math::Pose(leftPose.pos - this->resetPoseLeft.pos +
          this->basePoseLeft.pos,
          leftPose.rot * this->resetPoseLeft.rot.GetInverse() *
          this->basePoseLeft.rot);
  }
  /*else
  {
    this->basePoseLeft = leftAdjust;
  }*/

  this->SetRightFingers(_msg->right().trigger()*1.5707);
  this->SetLeftFingers(_msg->left().trigger()*1.5707);

  /*
  if (_msg->right().button_1())
    this->SetRightFingers(1.5707);
  else if (_msg->right().button_2())
    this->SetRightFingers(0);
  else if (_msg->right().button_4())
    this->SetRightFingers(-1.5707);

  if (_msg->left().button_1())
    this->SetLeftFingers(0);
  else if (_msg->left().button_2())
    this->SetLeftFingers(1.5707);
  else if (_msg->left().button_3())
    this->SetLeftFingers(-1.5707);
  */


  double dx = _msg->right().joy_x() * 0.002;
  double dy = _msg->right().joy_y() * -0.002;

  math::Pose dPose(dx, dy, 0, 0, 0, 0);
  this->pinJoint->Detach();
  this->model->SetWorldPose(this->model->GetWorldPose() + dPose);
  this->pinJoint->Attach(this->pinLink, this->model->GetLink("utorso"));

  rightAdjust += dPose;
  leftAdjust += dPose;

  this->rightModel->SetWorldPose(rightAdjust);
  this->leftModel->SetWorldPose(leftAdjust);

  this->basePoseRight += dPose;
  this->basePoseLeft += dPose;

  this->rightBumper = _msg->right().button_bumper();
  this->leftBumper = _msg->left().button_bumper();
}

void AtlasPlugin::Restart()
{
  this->rightModel->SetWorldPose(this->rightStartPose);
  this->leftModel->SetWorldPose(this->leftStartPose);
  this->basePoseRight = this->rightStartPose;
  this->basePoseLeft = this->leftStartPose;
  this->rightBumper = false;
  this->leftBumper = false;
  this->activated = false;

  this->pinJoint->Detach();
  this->model->SetWorldPose(this->modelStartPose);
  this->pinJoint->Attach(physics::LinkPtr(), this->model->GetLink("utorso"));

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

  this->dolly = this->world->GetModel("dolly");

  this->rightModel = this->world->GetModel("right_arm_goal");
  this->leftModel = this->world->GetModel("left_arm_goal");

  math::Quaternion modelRot = this->model->GetWorldPose().rot;

  this->basePoseRight = this->rightModel->GetWorldPose();
  this->basePoseLeft = this->leftModel->GetWorldPose();

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

    physics::ModelPtr pinModel = this->world->GetModel(pinModelStr);

    if (!pinModel)
      gzerr << "Unable to get pin model[" << pinModelStr << "]\n";
    else
    {
      this->pinLink = pinModel->GetLink(pinLinkStr);

      if (!this->pinLink)
        gzerr << "Unable to get pin link[" << pinLinkStr << "]\n";
    }
  }

  this->pinJoint->SetModel(this->model);

  this->pinJoint->Load(this->pinLink, this->model->GetLink("utorso"),
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

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&AtlasPlugin::Update, this, _1));
}

/////////////////////////////////////////////////
void AtlasPlugin::Update(const common::UpdateInfo & /*_info*/)
{
}
