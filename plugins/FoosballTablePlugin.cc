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

#include <algorithm>
#include <sdf/sdf.hh>
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "plugins/FoosballTablePlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(FoosballTablePlugin)

/////////////////////////////////////////////////
FoosballPlayer::FoosballPlayer(const std::string &_hydraTopic)
  : hydraTopic(_hydraTopic)
{
}

/////////////////////////////////////////////////
bool FoosballPlayer::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "FoosballPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "FoosballPlugin _sdf pointer is NULL");

  if (!_sdf->HasElement("team"))
  {
    std::cerr << "FoosballPlayer::Load() Missing <team> element in player "
              << "section" << std::endl;
    return false;
  }

  std::string team = _sdf->Get<std::string>("team");
  if (team != "Blue" && team != "Red")
  {
    std::cerr << "FoosballPlayer::Load() Invalid <team> value in player "
              << "section. Allowed values are 'Blue' or 'Red'" << std::endl;
    return false;
  }

  for (const auto &side : {"left_controller", "right_controller"})
  {
    if (_sdf->HasElement(side))
    {
      sdf::ElementPtr controllerElem = _sdf->GetElement(side);
      if (!controllerElem->HasElement("rod"))
      {
        std::cerr << "FoosballPlayer::Load() Missing <rod> element in <"
                  << side << "_controller> section" << std::endl;
        return false;
      }

      this->controller[side] = {};

      for (auto rodElem = controllerElem->GetElement("rod"); rodElem;
        rodElem = rodElem->GetNextElement("rod"))
      {
        std::string rodNumber = rodElem->GetValue()->GetAsString();
        std::string transName = "Foosball::trans" + team + rodNumber;
        std::string rotName = "Foosball::rot" + team + rodNumber;

        Rod_t rod = {_model->GetJoint(transName), _model->GetJoint(rotName)};
        this->controller[side].push_back(rod);
      }
    }
  }

  // Subscribe to Hydra updates by registering OnHydra() callback.
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(_model->GetWorld()->GetName());
  this->hydraSub = this->node->Subscribe(this->hydraTopic,
    &FoosballPlayer::OnHydra, this);

  return true;
}

/////////////////////////////////////////////////
void FoosballPlayer::Update()
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

    std::map<std::string, math::Pose> adjust;

    rightPose = msgs::Convert(msg->right().pose());
    leftPose = msgs::Convert(msg->left().pose());

    adjust["right_controller"] = math::Pose(
      rightPose.pos - this->resetPoseRight.pos + this->basePoseRight.pos,
      rightPose.rot * this->resetPoseRight.rot.GetInverse() *
      this->basePoseRight.rot);

    adjust["left_controller"] = math::Pose(
      leftPose.pos - this->resetPoseLeft.pos + this->basePoseLeft.pos,
      leftPose.rot * this->resetPoseLeft.rot.GetInverse() *
      this->basePoseLeft.rot);

    // Move the rods.
    for (const auto &side : {"left_controller", "right_controller"})
    {
      if (this->controller.find(side) != this->controller.end())
      {
        if (!this->controller[side].empty())
        {
          // Translation.
          this->controller[side].at(0).at(0)->SetPosition(0, -adjust[side].pos.x);
          // Rotation.
          this->controller[side].at(0).at(1)->SetPosition(
            0, -adjust[side].rot.GetRoll());
        }
      }
    }
  }
}


/////////////////////////////////////////////////
void FoosballPlayer::OnHydra(ConstHydraPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->msgMutex);

  if (_msg->right().button_center() &&_msg->left().button_center())
  {
    this->Restart();
    return;
  }

  if (_msg->left().trigger() > 0.2)
    this->leftTriggerPressed = true;
  else if (this->leftTriggerPressed)
  {
    this->leftTriggerPressed = false;
    this->SwitchRod(this->controller["left_controller"]);
  }

  if (_msg->right().trigger() > 0.2)
    this->rightTriggerPressed = true;
  else if (this->rightTriggerPressed)
  {
    this->rightTriggerPressed = false;
    this->SwitchRod(this->controller["right_controller"]);
  }

  this->hydraMsgs.push_back(_msg);
}

/////////////////////////////////////////////////
void FoosballPlayer::Restart()
{
  this->basePoseLeft = this->leftStartPose;
  this->basePoseRight = this->rightStartPose;
  this->activated = false;
}

/////////////////////////////////////////////////
void FoosballPlayer::SwitchRod(std::vector<Rod_t> &_aController)
{
  if (_aController.size() > 1)
  {
    std::rotate(_aController.begin(), _aController.begin() + 1,
      _aController.end());
  }
}

/////////////////////////////////////////////////
FoosballTablePlugin::FoosballTablePlugin()
{
}

/////////////////////////////////////////////////
void FoosballTablePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "FoosballPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "FoosballPlugin _sdf pointer is NULL");

  int counter = 0;
  for (auto playerElem = _sdf->GetElement("player"); playerElem;
    playerElem = playerElem->GetNextElement("player"))
  {
    std::string topic = "~/hydra" + std::to_string(counter);
    this->players.push_back(std::unique_ptr<FoosballPlayer>(
      new FoosballPlayer(topic)));
    this->players.at(counter)->Load(_model, playerElem);
    ++counter;
  }

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&FoosballTablePlugin::Update, this, _1));
}

/////////////////////////////////////////////////
void FoosballTablePlugin::Update(const common::UpdateInfo & /*_info*/)
{
  for (auto &player : this->players)
    player->Update();
}
