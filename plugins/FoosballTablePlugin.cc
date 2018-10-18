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
#include <mutex>
#include <string>
#include <sdf/sdf.hh>
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/math/Rand.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "plugins/FoosballTablePlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(FoosballTablePlugin)

/////////////////////////////////////////////////
FoosballPlayer::FoosballPlayer(const std::string &_hydraTopic)
  : hydraTopic(_hydraTopic),
    hydra({{"left_controller", {2}}, {"right_controller", {3}}}),
    lastHydraPose({{"left_controller", {0, 0}}, {"right_controller", {0, 0}}})
{
}

/////////////////////////////////////////////////
bool FoosballPlayer::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Initialize transport.
  this->gzNode = transport::NodePtr(new transport::Node());
  this->gzNode->Init();

  this->restartBallPub =
    this->gzNode->Advertise<msgs::Int>("~/foosball_demo/restart_ball");

  this->visualPub = this->gzNode->Advertise<msgs::Visual>("~/visual");

  GZ_ASSERT(_model, "FoosballPlugin _model pointer is NULL");
  this->model = _model;
  GZ_ASSERT(_sdf, "FoosballPlugin _sdf pointer is NULL");

  // Read the <team> element.
  if (!_sdf->HasElement("team"))
  {
    std::cerr << "FoosballPlayer::Load() Missing <team> element in player "
              << "section" << std::endl;
    return false;
  }
  this->team = _sdf->Get<std::string>("team");
  if (this->team != "Blue" && this->team != "Red")
  {
    std::cerr << "FoosballPlayer::Load() Invalid <team> value in player "
              << "section. Allowed values are 'Blue' or 'Red'" << std::endl;
    return false;
  }

  // Fill the vector of rods.
  for (unsigned int i = 0; i < this->kNumRodsPerTeam; ++i)
  {
    std::string transName = "Foosball::trans" + this->team + std::to_string(i);
    std::string rotName = "Foosball::rot" + this->team + std::to_string(i);

    // Create a new rod and make it controllable by this controller.
    this->rods.push_back(
      {_model->GetJoint(transName), _model->GetJoint(rotName)});

    // Visuals
    std::string shaftName =
        "foosball::Foosball::shaft" + this->team + std::to_string(i);
    this->shafts.push_back(shaftName);
  }

  // Check if we have to invert the control for this player.
  if (_sdf->HasElement("invert_control"))
  {
    int invertControl = _sdf->Get<int>("invert_control");
    if (invertControl == 1)
    {
      this->invert = -1.0;

      // Swap left/right controllers.
      auto &leftRods = this->hydra["left_controller"];
      auto &rightRods = this->hydra["right_controller"];
      std::swap(leftRods, rightRods);
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

  if (!this->hydraMsgPtr)
    return;

  // Read the last known Hydra message.
  boost::shared_ptr<msgs::Hydra const> msg = this->hydraMsgPtr;

  // Reset the Hydra position.
  if (msg->right().button_center() && msg->left().button_center())
  {
    this->activated = true;
    this->resetPoseRight = msgs::Convert(msg->right().pose());
    this->resetPoseLeft = msgs::Convert(msg->left().pose());
  }

  // Restart the ball position.
  if (msg->left().trigger() > 0.5 || msg->right().trigger() > 0.5)
    this->restartBallPending = true;
  else if (this->restartBallPending)
  {
    msgs::Int restartBallMsg;
    restartBallMsg.set_data(1);
    this->restartBallPub->Publish(restartBallMsg);
    this->restartBallPending = false;
  }

  if (this->activated)
  {
    // If the user pressed the trigger we have to change the active rod.
    this->UpdateActiveRod();

    math::Pose rightPose = msgs::Convert(msg->right().pose());
    math::Pose leftPose = msgs::Convert(msg->left().pose());

    std::map<std::string, math::Pose> adjust;
    adjust["right_controller"] = math::Pose(
      rightPose.pos - this->resetPoseRight.pos,
      rightPose.rot * this->resetPoseRight.rot.GetInverse());
    adjust["left_controller"] = math::Pose(
      leftPose.pos - this->resetPoseLeft.pos,
      leftPose.rot * this->resetPoseLeft.rot.GetInverse());

    common::Time curTime = this->model->GetWorld()->GetSimTime();
    if (curTime > this->lastUpdateTime)
    {
      common::Time dt = (curTime - this->lastUpdateTime).Double();

      // Move the rods.
      for (auto const &side : {"left_controller", "right_controller"})
      {
        // Get the active rod.
        auto &activeRod = this->rods[this->hydra[side]];

        // Translation.
        double vel = (adjust[side].pos.x - this->lastHydraPose[side].at(0))
          / dt.Double();
        activeRod.at(0)->SetVelocity(0, this->invert * vel);

        // Rotation.
        auto dRot = GZ_NORMALIZE(adjust[side].rot.GetRoll() -
          this->lastHydraPose[side].at(1));
        vel = dRot / dt.Double();
        activeRod.at(1)->SetVelocity(0, this->invert * vel);

        // Store the current Hydra position for calculating the velocity
        // of Hydra in the next iteration.
        this->lastHydraPose[side].at(0) = adjust[side].pos.x;
        this->lastHydraPose[side].at(1) = adjust[side].rot.GetRoll();

        // Store the current rod positions in case we have to switch rods.
        // If we switch a rod we have to put the new rod in the same
        // position as the previous one.
        this->lastRodPose[side].at(0) = activeRod.at(0)->GetAngle(0);
        this->lastRodPose[side].at(1) = activeRod.at(1)->GetAngle(0);
      }
    }

    this->lastUpdateTime = this->model->GetWorld()->GetSimTime();
  }
}

/////////////////////////////////////////////////
void FoosballPlayer::OnHydra(ConstHydraPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->msgMutex);
  this->hydraMsgPtr = _msg;
}

/////////////////////////////////////////////////
void FoosballPlayer::UpdateActiveRod()
{
  if (!this->hydraMsgPtr)
    return;

  // Read the last known Hydra message.
  boost::shared_ptr<msgs::Hydra const> msg = this->hydraMsgPtr;

  if (std::abs(msg->left().joy_y()) > 0.5)
  {
    this->pendingRodChange = true;
    this->leftJoyCmd = msg->left().joy_y();
  }
  if (std::abs(msg->right().joy_y()) > 0.5)
  {
    this->pendingRodChange = true;
    this->rightJoyCmd = msg->right().joy_y();
  }

  // Both joysticks are back to normal position and there's a pending rod change
  if ((std::abs(msg->left().joy_y()) <= 0.5) &&
      (std::abs(msg->right().joy_y()) <= 0.5) &&
      (this->pendingRodChange))
  {
    this->SwitchRod(this->leftJoyCmd, this->rightJoyCmd);
    this->pendingRodChange = false;
    this->leftJoyCmd = 0.0;
    this->rightJoyCmd = 0.0;
  }
}

/////////////////////////////////////////////////
void FoosballPlayer::SwitchRod(const double _leftDir, const double _rightDir)
{
  unsigned int left = this->hydra["left_controller"];
  unsigned int right = this->hydra["right_controller"];
  unsigned newLeft, newRight;
  bool leftChanged = false, rightChanged = false;

  // Left controller moving left.
  if (_leftDir < -0.5 && left > 0)
  {
    newLeft = left - 1;
    leftChanged = true;
  }

  // Left controller moving right.
  if ((_leftDir > 0.5 && (right - left > 1)) ||
      (_leftDir > 0.5 && (right - left == 1) && _rightDir > 0.5 &&
        right < kNumRodsPerTeam - 1))
  {
    newLeft = left + 1;
    leftChanged = true;
  }

  // Right controller moving left.
  if ((_rightDir < -0.5 && (right - left > 1)) ||
      (_rightDir < -0.5 && (right - left == 1) && _leftDir < -0.5 &&
        left > 0))
  {
    newRight = right - 1;
    rightChanged = true;
  }

  // Right controller moving right.
  if (_rightDir > 0.5 && right < kNumRodsPerTeam - 1)
  {
    newRight = right + 1;
    rightChanged = true;
  }

  // Restore the position/orientation of the new left rod.
  if (leftChanged)
  {
    // Reset previous active rod's color
    std::string name =
        this->shafts[this->hydra["left_controller"]] + "::handle";
    std::string parentName = this->shafts[this->hydra["left_controller"]];
    std::string color = "Gazebo/Black";
    this->PublishVisualMsg(name, parentName, color);

    // Update index
    this->hydra["left_controller"] = newLeft;

    // Restore the position to the last known Hydra position.
    auto &activeRod = this->rods[this->hydra["left_controller"]];
    activeRod.at(0)->SetPosition(0,
      this->lastRodPose["left_controller"].at(0).Radian());
    activeRod.at(1)->SetPosition(0,
      this->lastRodPose["left_controller"].at(1).Radian());

    // Publish active rod msg
    name = this->shafts[this->hydra["left_controller"]] + "::handle";
    parentName = this->shafts[this->hydra["left_controller"]];
    color = "Gazebo/" + this->team;
    this->PublishVisualMsg(name, parentName, color);
  }

  // Restore the position/orientation of the new right rod.
  if (rightChanged)
  {
    // Reset previous active rod's color
    std::string name =
        this->shafts[this->hydra["right_controller"]] + "::handle";
    std::string parentName = this->shafts[this->hydra["right_controller"]];
    std::string color = "Gazebo/Black";
    this->PublishVisualMsg(name, parentName, color);

    // Update index
    this->hydra["right_controller"] = newRight;

    // Restore the position to the last known Hydra position.
    auto &activeRod = this->rods[this->hydra["right_controller"]];
    activeRod.at(0)->SetPosition(0,
      this->lastRodPose["right_controller"].at(0).Radian());
    activeRod.at(1)->SetPosition(0,
      this->lastRodPose["right_controller"].at(1).Radian());

    // Publish active rod msg
    name = this->shafts[this->hydra["right_controller"]] + "::handle";
    parentName = this->shafts[this->hydra["right_controller"]];
    color = "Gazebo/" + this->team;
    this->PublishVisualMsg(name, parentName, color);
  }
}

/////////////////////////////////////////////////
void FoosballPlayer::PublishVisualMsg(std::string &_name,
    std::string &_parentName, std::string &_color)
{
  msgs::Visual visualMsg;
  visualMsg.set_name(_name);
  visualMsg.set_parent_name(_parentName);
  visualMsg.mutable_material()->mutable_script()->set_name(_color);

  this->visualPub->Publish(visualMsg);
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
  this->model = _model;

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

  // Subscribe to shake table
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(_model->GetWorld()->GetName());
  this->shakeTableSub =
    this->node->Subscribe("~/foosball_demo/shake_table",
    &FoosballTablePlugin::OnShakeTable, this);
}

/////////////////////////////////////////////////
void FoosballTablePlugin::Update(const common::UpdateInfo & /*_info*/)
{
  for (auto &player : this->players)
    player->Update();
}

/////////////////////////////////////////////////
void FoosballTablePlugin::OnShakeTable(ConstIntPtr &/*_unused*/)
{
  if (!this->model || !this->model->GetLink("Foosball::table"))
    return;

  double randX = math::Rand::GetDblUniform(-1, 1);
  double randY = math::Rand::GetDblUniform(-1, 1);
  double randZ = math::Rand::GetDblUniform(0, 1);

  this->model->GetLink("Foosball::table")->AddLinkForce(
      math::Vector3(0, 0, 10000*randZ));

  this->model->GetLink("Foosball::table")->AddRelativeTorque(
      math::Vector3(10000*randX, 10000*randY, 0));
}

