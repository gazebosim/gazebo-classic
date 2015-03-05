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
#include <string>
#include <sdf/sdf.hh>
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportIface.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "plugins/FoosballDemoPlugin.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(FoosballDemoPlugin)

/////////////////////////////////////////////////
FoosballDemoPlugin::FoosballDemoPlugin()
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

  // Initialize game duration.
  if (_sdf->HasElement("game_time"))
    this->gameDuration = gazebo::common::Time(_sdf->Get<int>("game_time"), 0);
  else
    this->gameDuration = gazebo::common::Time(kDefaultGameTime, 0);

  // Initialize transport.
  this->gzNode = transport::NodePtr(new transport::Node());
  this->gzNode->Init();
  this->timePub = this->gzNode->Advertise<msgs::Time>("~/foosball_demo/time");
  this->scorePub =
    this->gzNode->Advertise<msgs::GzString>("~/foosball_demo/score");

  // Connect to the update event.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&FoosballDemoPlugin::Update, this, _1));

  this->RestartGame();
}

/////////////////////////////////////////////////
void FoosballDemoPlugin::Reset()
{
  this->RestartGame();
}

/////////////////////////////////////////////////
void FoosballDemoPlugin::Update(const common::UpdateInfo &/*_info*/)
{
  // Check for goals.
  math::Pose ballPose = this->ball->GetWorldPose();

  // If goal, update score and restart ball.

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
}

/////////////////////////////////////////////////
void FoosballDemoPlugin::RestartGame()
{
  this->startTimeSim = this->world->GetSimTime();
  this->gameTime = this->gameDuration;
  scoreA = scoreB = 0;
}

/////////////////////////////////////////////////
void FoosballDemoPlugin::RestartBall()
{
  math::Pose newPose(math::Pose(0, 0, 0.2, 0, 0, 0));
  math::Vector3 newVel(0, 0.5, -0.2);
  this->ball->SetWorldPose(newPose);
  this->ball->SetLinearVel(newVel);
}
