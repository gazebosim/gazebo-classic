/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "plugins/FoosballPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(FoosballPlugin)

/////////////////////////////////////////////////
FoosballPlugin::FoosballPlugin()
{
}

/////////////////////////////////////////////////
void FoosballPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "FoosballPlugin _model pointer is NULL");
  this->model = _model;
  GZ_ASSERT(_sdf, "FoosballPlugin _sdf pointer is NULL");
  this->sdf = _sdf;

  this->leftModel = this->model->GetJoint("Foosball::rotA0");
  if (!this->leftModel)
    gzerr << "Unable to get Foosball::shaftA0 joint\n";

  this->rightModel = this->model->GetJoint("Foosball::rotA2");
  if (!this->rightModel)
    gzerr << "Unable to get Foosball::shaftA2 joint\n";

  // Subscribe to Hydra updates by registering OnHydra() callback.
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());
  this->hydraSub = this->node->Subscribe("~/hydra",
    &FoosballPlugin::OnHydra, this);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&FoosballPlugin::Update, this, _1));
}

/////////////////////////////////////////////////
void FoosballPlugin::Update(const common::UpdateInfo & /*_info*/)
{
  std::lock_guard<std::mutex> lock(this->msgMutex);

  if (this->hydraMsgs.empty())
    return;

  if (!this->activated)
  {
    for (std::list<boost::shared_ptr<msgs::Hydra const> >::iterator iter =
        this->hydraMsgs.begin(); iter != this->hydraMsgs.end(); ++iter)
    {
      if ((*iter)->right().button_bumper() && (*iter)->left().button_bumper())
      {
        std::cout << "Activated!" << std::endl;
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

    rightAdjust = math::Pose(rightPose.pos - this->resetPoseRight.pos +
        this->basePoseRight.pos,
        rightPose.rot * this->resetPoseRight.rot.GetInverse() *
        this->basePoseRight.rot);

    leftAdjust = math::Pose(leftPose.pos - this->resetPoseLeft.pos +
        this->basePoseLeft.pos,
        leftPose.rot * this->resetPoseLeft.rot.GetInverse() *
        this->basePoseLeft.rot);

  }
}

/////////////////////////////////////////////////
void FoosballPlugin::OnHydra(ConstHydraPtr &_msg)
{
  if (_msg->right().button_center() &&_msg->left().button_center())
  {
    this->Restart();
    return;
  }

  std::lock_guard<std::mutex> lock(this->msgMutex);
  this->hydraMsgs.push_back(_msg);
}

/////////////////////////////////////////////////
void FoosballPlugin::Restart()
{
  std::cout << "Restart!" << std::endl;

  this->basePoseLeft = this->leftStartPose;
  this->basePoseRight = this->rightStartPose;
  this->activated = false;
}
