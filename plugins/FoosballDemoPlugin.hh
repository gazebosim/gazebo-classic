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

#ifndef _GAZEBO_FOOSBALL_DEMO_PLUGIN_HH_
#define _GAZEBO_FOOSBALL_DEMO_PLUGIN_HH_

#include <sdf/sdf.hh>
#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  /// \brief A world plugin that...
  class GAZEBO_VISIBLE FoosballDemoPlugin : public WorldPlugin
  {
    /// \brief Constructor.
    public: FoosballDemoPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    // Documentation inherited.
    public: virtual void Reset();

    /// \brief Update the state of the game: time, score, ball position in case
    /// of goals.
    /// \param[in] _info Information used in the update event.
    private: void Update(const common::UpdateInfo &_info);

    /// \brief Restart a game.
    private: void RestartGame();

    /// \brief Restart ball.
    private: void RestartBall();

    /// \brief Default game time in seconds.
    private: int kDefaultGameTime = 180;

    /// \brief World pointer.
    private: physics::WorldPtr world;

    /// \brief SDF for this plugin.
    private: sdf::ElementPtr sdf;

    /// \brief Pointer to the ball.
    private: physics::ModelPtr ball;

    /// \brief Pointer to a node for communication.
    private: transport::NodePtr gzNode;

    /// \brief Time publisher.
    private: transport::PublisherPtr timePub;

    /// \brief Score publisher.
    private: transport::PublisherPtr scorePub;

    /// \brief Pointer to the update event connection.
    private: event::ConnectionPtr updateConnection;

    /// \brief Sim time at which the game started.
    private: common::Time startTimeSim;

    /// \brief Game duration.
    private: common::Time gameDuration;

    /// \brief Seconds remaining to finish the game.
    private: common::Time gameTime;

    /// \brief Player "A" score.
    private: int scoreA;

    /// \brief Player "B" score.
    private: int scoreB;
  };
}
#endif
