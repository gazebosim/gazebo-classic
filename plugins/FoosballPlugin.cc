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

  for (auto playerElem = _sdf->GetElement("player"); playerElem;
    playerElem = playerElem->GetNextElement("player"))
  {
    if (!playerElem->HasElement("name"))
    {
      std::cerr << "FoosballPlugin::Load() Missing <name> element in player "
                << "section" << std::endl;
      continue;
    }
    std::string name = playerElem->Get<std::string>("name");
    if (name != "A" && name != "B")
    {
      std::cerr << "FoosballPlugin::Load() Invalid <name> value in player "
                << "section. Allowed values ar A or B" << std::endl;
    }

    Controller_t controller;

    for (const auto &side : {"left_controller", "right_controller"})
    {
      if (playerElem->HasElement(side))
      {
        sdf::ElementPtr controllerElem = playerElem->GetElement(side);
        if (!controllerElem->HasElement("rod"))
        {
          std::cerr << "FoosballPlugin::Load() Missing <rod> element in <"
                    << side << "_controller> section" << std::endl;
          continue;
        }

        controller[side] = {};

        for (auto rodElem = controllerElem->GetElement("rod"); rodElem;
          rodElem = rodElem->GetNextElement("rod"))
        {
          std::string rodNumber = rodElem->GetValue()->GetAsString();
          std::string transName = "Foosball::trans" + name + rodNumber;
          std::string rotName = "Foosball::rot" + name + rodNumber;
          Rod_t rod =
           {this->model->GetJoint(transName), this->model->GetJoint(rotName)};

           controller[side].push_back(rod);
        }
      }
    }

    std::cout << "Player " << name << " detected" << std::endl;
    for (const auto &side : controller)
    {
      std::cout << "\t" << side.first << ":" << std::endl;
      for (const auto &rod : side.second)
      {
        std::cout << "\t\tTranslation: " << rod.at(0)->GetName() << std::endl;
        std::cout << "\t\tRotation: " << rod.at(1)->GetName() << std::endl;
      }
    }

    this->controllers.push_back(controller);
  }

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
    for (auto &controller : this->controllers)
    {
      for (const auto &side : {"left_controller", "right_controller"})
      {
        if (controller.find(side) != controller.end())
        {
          if (!controller[side].empty())
          {
            // Translation.
            controller[side].at(0).at(0)->SetPosition(0, -adjust[side].pos.x);
            // Rotation.
            controller[side].at(0).at(1)->SetPosition(
              0, -adjust[side].rot.GetRoll());
          }
        }
      }
    }
  }
}

/////////////////////////////////////////////////
void FoosballPlugin::SwitchRod(std::vector<Rod_t> &_aController)
{
  if (_aController.size() > 1)
  {
    std::rotate(_aController.begin(), _aController.begin() + 1,
      _aController.end());
  }
}

/////////////////////////////////////////////////
void FoosballPlugin::OnHydra(ConstHydraPtr &_msg)
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
    this->SwitchRod(this->controllers.at(0)["left_controller"]);
  }

  if (_msg->right().trigger() > 0.2)
    this->rightTriggerPressed = true;
  else if (this->rightTriggerPressed)
  {
    this->rightTriggerPressed = false;
    this->SwitchRod(this->controllers.at(0)["right_controller"]);
  }

  this->hydraMsgs.push_back(_msg);
}

/////////////////////////////////////////////////
void FoosballPlugin::Restart()
{
  this->basePoseLeft = this->leftStartPose;
  this->basePoseRight = this->rightStartPose;
  this->activated = false;
}
