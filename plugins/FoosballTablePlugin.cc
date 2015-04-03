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
#include <mutex>
#include <string>
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
  : hydraTopic(_hydraTopic),
    hydra({{"left_controller", {}}, {"right_controller", {}}})
{
  for (const auto &side : {"left_controller", "right_controller"})
  {
    this->hydraPID[side] = {common::PID(), common::PID()};
    for (auto i = 0; i < 2; ++i)
      this->hydraPID[side].at(i).Init(1.0, 0.0, 0.5, 0.0, 0.0, 60.0, -60.0);
  }
}

/////////////////////////////////////////////////
bool FoosballPlayer::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "FoosballPlugin _model pointer is NULL");
  this->model = _model;
  GZ_ASSERT(_sdf, "FoosballPlugin _sdf pointer is NULL");

  if (!_sdf->HasElement("team"))
  {
    std::cerr << "FoosballPlayer::Load() Missing <team> element in player "
              << "section" << std::endl;
    return false;
  }

  std::string team = _sdf->Get<std::string>("team");
  if (team != "blue" && team != "red")
  {
    std::cerr << "FoosballPlayer::Load() Invalid <team> value in player "
              << "section. Allowed values are 'blue' or 'red'" << std::endl;
    return false;
  }

  for (const auto &side : {"left_controller", "right_controller"})
  {
    if (_sdf->HasElement(side))
    {
      sdf::ElementPtr controllerElem = _sdf->GetElement(side);
      if (!controllerElem->HasElement("rod"))
      {
        std::cerr << "FoosballPlayer::Load() Missing <rod> element in "
                  << "<" << side << "> section" << std::endl;
        return false;
      }

      for (auto rodElem = controllerElem->GetElement("rod"); rodElem;
        rodElem = rodElem->GetNextElement("rod"))
      {
        std::string transName;
        std::string rotName;
        std::string rodNumber = rodElem->GetValue()->GetAsString();

        if (team == "red")
        {
          transName = "Foosball::transA" + rodNumber;
          rotName = "Foosball::rotA" + rodNumber;
        }
        else
        {
          transName = "Foosball::transB" + rodNumber;
          rotName = "Foosball::rotB" + rodNumber;
        }

        // Create a new rod and make it controllable by this controller.
        Rod_t rod = {_model->GetJoint(transName), _model->GetJoint(rotName)};
        this->hydra[side].push_back(rod);
      }
    }
  }

  // Subscribe to Hydra updates by registering OnHydra() callback.
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(_model->GetWorld()->GetName());
  this->hydraSub = this->node->Subscribe(this->hydraTopic,
    &FoosballPlayer::OnHydra, this);

  this->lastUpdateTime = this->model->GetWorld()->GetSimTime();

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

    common::Time curTime = this->model->GetWorld()->GetSimTime();

    if (curTime > this->lastUpdateTime)
    {
      common::Time dt = (curTime - this->lastUpdateTime).Double();

      // Move the rods.
      for (const auto &side : {"left_controller", "right_controller"})
      {
        if (this->hydra.find(side) != this->hydra.end())
        {
          if (!this->hydra[side].empty())
          {
            // Get the current rod. The first element of the vector of rods is
            // always the active rod.
            auto &currentRod = this->hydra[side].at(0);

            // Translation.
            //   Get the target x position based on the Hydra controller.
            double target = -adjust[side].pos.x;

            //   Get the current x rod position.
            double current =  currentRod.at(0)->GetWorldPose().pos.x;

            //   Position error between the rod and the Hydra controller.
            double error = current - target;

            //   Update the PID.
            double torque = this->hydraPID[side].at(0).Update(error, dt);
            currentRod.at(0)->SetForce(0, torque);

            // Rotation.
            //   Get the target angle based on the Hydra controller.
            target = -adjust[side].rot.GetRoll();

            //   Get the current rod angle.
            current = currentRod.at(1)->GetAngle(0).Radian();

            //   Angle error between the rod and Hydra controller.
            error = current - target;

            //   Update the PID.
            torque = this->hydraPID[side].at(1).Update(error, dt);
            currentRod.at(1)->SetForce(0, torque);
          }
        }
      }
    }

    this->lastUpdateTime = this->model->GetWorld()->GetSimTime();
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
    this->SwitchRod(this->hydra["left_controller"]);
  }

  if (_msg->right().trigger() > 0.2)
    this->rightTriggerPressed = true;
  else if (this->rightTriggerPressed)
  {
    this->rightTriggerPressed = false;
    this->SwitchRod(this->hydra["right_controller"]);
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
void FoosballPlayer::SwitchRod(Rod_V &_rods)
{
  if (_rods.size() > 1)
  {
    std::rotate(_rods.begin(), _rods.begin() + 1, _rods.end());
  }
}

/////////////////////////////////////////////////
FoosballTablePlugin::~FoosballTablePlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
}

/////////////////////////////////////////////////
void FoosballTablePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "FoosballPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "FoosballPlugin _sdf pointer is NULL");

  // Create the players.
  int counter = 0;
  for (auto playerElem = _sdf->GetElement("player"); playerElem;
    playerElem = playerElem->GetNextElement("player"))
  {
    std::string topic = "~/hydra" + std::to_string(counter);
    this->players.push_back(
      std::unique_ptr<FoosballPlayer>(new FoosballPlayer(topic)));
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
