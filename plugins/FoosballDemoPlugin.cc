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
#include <iostream>
#include <mutex>
#include <string>
#include <sdf/sdf.hh>
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/math/Pose.hh"
#include "gazebo/math/Rand.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportIface.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "plugins/FoosballDemoPlugin.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(FoosballDemoPlugin)

/////////////////////////////////////////////////
void KickoffState::Initialize()
{
  State::Initialize();

  // Move the ball to the centre of the table.
  float ballHeight = this->plugin->tableHeight + 0.2;
  math::Pose newPose(math::Pose(0, 0, ballHeight, 0, 0, 0));
  this->plugin->ball->SetWorldPose(newPose);
  this->plugin->ball->ResetPhysicsStates();
}

/////////////////////////////////////////////////
void KickoffState::Update()
{
  // After some time, go to play mode.
  common::Time elapsed = this->timer.GetElapsed();
  if (elapsed.sec > 2)
    this->plugin->SetCurrentState(this->plugin->playState);
}

/////////////////////////////////////////////////
GoalState::GoalState(const std::string &_name, FoosballDemoPlugin *_plugin,
  int *_score)
  : State(_name, _plugin),
    score(_score)
{
}

/////////////////////////////////////////////////
void GoalState::Initialize()
{
  State::Initialize();
  (*this->score)++;
}

/////////////////////////////////////////////////
void GoalState::Update()
{
  // After some time, go to kickoff mode.
  common::Time elapsed = this->timer.GetElapsed();
  if (elapsed.sec > 2)
    this->plugin->SetCurrentState(this->plugin->kickoffState);
}

/////////////////////////////////////////////////
void PlayState::Initialize()
{
  State::Initialize();

  // Launch the ball and start the game!
  double randomOffset = math::Rand::GetDblUniform(-0.1, 0.1);
  math::Vector3 newVel(randomOffset, 0.5, -0.2);
  this->plugin->ball->SetLinearVel(newVel);
}

/////////////////////////////////////////////////
void PlayState::Update()
{
  // Is the game done?
  if (this->plugin->gameTime == 0)
  {
    this->plugin->SetCurrentState(this->plugin->finishedState);
    return;
  }

  math::Pose ballPose = this->plugin->ball->GetWorldPose();

  // Does player "A" score?
  if (ballPose.pos.x > this->plugin->tableLength / 2.0)
  {
    this->plugin->SetCurrentState(this->plugin->goalAState);
    return;
  }

  // Does player "B" score?
  if (ballPose.pos.x < -this->plugin->tableLength / 2.0)
  {
    this->plugin->SetCurrentState(this->plugin->goalBState);
    return;
  }
}

/////////////////////////////////////////////////
void FinishedState::Initialize()
{
  State::Initialize();

  // Move the ball to the centre of the table.
  float ballHeight = this->plugin->tableHeight + 0.2;
  math::Pose newPose(math::Pose(0, 0, ballHeight, 0, 0, 0));
  this->plugin->ball->SetWorldPose(newPose);
  this->plugin->ball->ResetPhysicsStates();
}

/////////////////////////////////////////////////
void FinishedState::Update()
{
  // Stay here forever unless someone restarts the game.
}

/////////////////////////////////////////////////
FoosballDemoPlugin::~FoosballDemoPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
}

/////////////////////////////////////////////////
void FoosballDemoPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  if (!_world)
  {
    std::cerr << "FoosballDemoPlugin _world pointer is NULL" << std::endl;
    return;
  }
  this->world = _world;

  this->sdf = _sdf;

  // Get a pointer to the ball.
  this->ball = this->world->GetModel(_sdf->Get<std::string>("ball"));
  if (!this->ball)
  {
    std::cerr << "Unable to find the foosball ball model [" <<
      _sdf->Get<std::string>("ball") << "]" << std::endl;
    return;
  }

  // Read the table length.
  if (!_sdf->HasElement("table_length"))
  {
    std::cerr << "Unable to find the [table_length] parameter" << std::endl;
    return;
  }
  this->tableLength = _sdf->Get<float>("table_length");

  // Read the table height.
  if (!_sdf->HasElement("table_height"))
  {
    std::cerr << "Unable to find the [table_height] parameter" << std::endl;
    return;
  }
  this->tableHeight = _sdf->Get<float>("table_height");

  // Initialize game duration.
  if (_sdf->HasElement("game_duration"))
    this->gameDuration = common::Time(_sdf->Get<int>("game_duration"), 0);
  else
  {
    std::cerr << "Unable to find the [game_duration] parameter."
                 " Using the default duration" << std::endl;
    this->gameDuration = common::Time(kDefaultGameTime, 0);
  }

  // Initialize transport.
  this->gzNode = transport::NodePtr(new transport::Node());
  this->gzNode->Init();
  this->timePub = this->gzNode->Advertise<msgs::Time>("~/foosball_demo/time");
  this->scorePub =
    this->gzNode->Advertise<msgs::GzString>("~/foosball_demo/score");
  this->statePub =
    this->gzNode->Advertise<msgs::GzString>("~/foosball_demo/state");
  this->restartBallSub =
    this->gzNode->Subscribe("~/foosball_demo/restart_ball",
    &FoosballDemoPlugin::OnRestartBall, this);
  this->restartGameSub =
    this->gzNode->Subscribe("~/foosball_demo/restart_game",
    &FoosballDemoPlugin::OnRestartGame, this);

  // Connect to the update event.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&FoosballDemoPlugin::Update, this, _1));

  this->OnRestartGame(nullptr);
}

/////////////////////////////////////////////////
void FoosballDemoPlugin::Reset()
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->OnRestartGame(nullptr);
}

/////////////////////////////////////////////////
void FoosballDemoPlugin::Update(const common::UpdateInfo &/*_info*/)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  // Step.
  this->currentState->Update();

  // Update and publish game time.
  common::Time elapsed = this->world->GetSimTime() - this->startTimeSim;
  this->gameTime = std::max(0.0, (this->gameDuration - elapsed).Double());
  this->timePub->Publish(msgs::Convert(this->gameTime));

  // Publish score.
  std::string score =
    std::to_string(this->scoreA) + ":" + std::to_string(this->scoreB);
  msgs::GzString scoreMsg;
  scoreMsg.set_data(score);
  this->scorePub->Publish(scoreMsg);

  // Publish game state.
  msgs::GzString stateMsg;
  stateMsg.set_data(this->currentState->GetName());
  this->statePub->Publish(stateMsg);
}

/////////////////////////////////////////////////
void FoosballDemoPlugin::SetCurrentState(State<FoosballDemoPlugin> &_newState)
{
  // Only update the state if _newState is different than the current state.
  if (this->currentState->GetName() != _newState.GetName())
  {
    this->currentState = &_newState;
    this->currentState->Initialize();
  }
}

/////////////////////////////////////////////////
void FoosballDemoPlugin::OnRestartBall(ConstIntPtr &/*_unused*/)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  this->currentState = &kickoffState;
  this->currentState->Initialize();
}

/////////////////////////////////////////////////
void FoosballDemoPlugin::OnRestartGame(ConstIntPtr &/*_unused*/)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  this->startTimeSim = this->world->GetSimTime();
  this->gameTime = this->gameDuration;
  scoreA = scoreB = 0;

  this->currentState = &kickoffState;
  this->currentState->Initialize();
}
