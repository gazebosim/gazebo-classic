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

#include <mutex>
#include <string>
#include <sdf/sdf.hh>
#include "gazebo/common/Time.hh"
#include "gazebo/common/Timer.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  /// \class State FoosballDemoPlugin.hh
  // \brief State pattern used for the game mode.
  template<typename T> class State
  {
    /// \brief Class constructor.
    /// \param[in] _name Name of the state.
    /// \param[in] _plugin A plugin pointer.
    public: State(const std::string &_name, T *_plugin)
      : name(_name),
        plugin(_plugin)
    {
    }

    /// \brief Class destructor.
    public: virtual ~State()
    {
    }

    /// \brief Initialize the state. Called once when the state is entered.
    public: virtual void Initialize()
    {
      this->timer.Reset();
      this->timer.Start();
    }

    /// \brief Update the state.
    public: virtual void Update() = 0;

    /// \brief Get the name of the state.
    /// \return Returns the name of the state.
    public: std::string GetName()
    {
      return this->name;
    }

    /// \brief Timer to measure the time elapsed in this state.
    public: common::Timer timer;

    /// \brief Name of the state.
    protected: std::string name;

    /// \brief Pointer to be able to access to the plugin inside the state.
    protected: T *plugin;
  };

  // Forward declaration.
  class FoosballDemoPlugin;

  /// \class KickoffState FoosballDemoPlugin.hh
  /// \brief State for the kickoff period.
  class KickoffState : public State<FoosballDemoPlugin>
  {
    /// Inherit constructor from State.
    using State<FoosballDemoPlugin>::State;

    /// Documentation inherited.
    public: virtual void Initialize();

    // Documentation inherited
    public: virtual void Update();
  };

  /// \class GoalState FoosballDemoPlugin.hh
  /// \brief State for representing a goal scored by a team.
  class GoalState : public State<FoosballDemoPlugin>
  {
    /// \brief Class constructor.
    /// \param _score Pointer to the variable that stores the number of goals.
    public: GoalState(const std::string &_name, FoosballDemoPlugin *_plugin,
                      int *_score);

    /// Documentation inherited.
    public: virtual void Initialize();

    // Documentation inherited
    public: virtual void Update();

    /// \brief Pointer to the score managed by this object.
    private: int *score;
  };

  /// \class playState FoosballDemoPlugin.hh
  /// \brief State for representing the regular play mode.
  class PlayState : public State<FoosballDemoPlugin>
  {
    /// Inherit constructor from State.
    using State<FoosballDemoPlugin>::State;

    /// Documentation inherited.
    public: virtual void Initialize();

    // Documentation inherited
    public: virtual void Update();
  };

  /// \class finishedState FoosballDemoPlugin.hh
  /// \brief State for representing the end of the game.
  class FinishedState : public State<FoosballDemoPlugin>
  {
    /// Inherit constructor from State.
    using State<FoosballDemoPlugin>::State;

    /// Documentation inherited.
    public: virtual void Initialize();

    // Documentation inherited
    public: virtual void Update();
  };

  /// \brief A world plugin that implements a foosball game.
  /// It assumes that a foosball table is in the world at coordinates (0 0 0)
  /// with its longer side aligned with the global X axis.
  ///
  /// The plugin accepts the following parameters:
  ///   * [table_length]  : Length of the foosball table (m).
  ///   * [table_height]  : Height of the foosball table (m).
  ///   * [game_duration] : Duration of each game (secs).
  ///
  /// The game has the following states:
  ///   * Kickoff : The ball is still in the center of the table.
  ///   * Play    : The ball is thrown and the game starts.
  ///   * GoalA   : Player "A" scores a goal. After a few seconds the game
  ///               transitions to Kickoff state.
  ///   * GoalB   : Similar to the previous state but triggered when "B" scores.
  ///   * Finished: End of the game.
  ///
  /// The plugin periodically publishes the following information:
  ///   * Game time [~/foosball_demo/time] : Remaining game time (s).
  ///   * Score [~/foosball_demo/score] : The format is "<goals_A>:<goals_B>".
  ///   * State [~/foosball_demo/state] : Game state in string format.
  ///
  /// The plugin can receive two messages that triggers the following actions:
  ///   * Ball restart [~/foosball_demo/ball_restart] : Restart the ball.
  ///   * Game restart [~/foosball_demo/game_restart] : Restart the game.
  class GAZEBO_VISIBLE FoosballDemoPlugin : public WorldPlugin
  {
    /// \brief Constructor.
    public: FoosballDemoPlugin() = default;

    /// \brief Destructor.
    public: virtual ~FoosballDemoPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    // Documentation inherited.
    public: virtual void Reset();

    /// \brief Update the state of the game: time, score, ball position in case
    /// of goals.
    /// \param[in] _info Information used in the update event.
    private: void Update(const common::UpdateInfo &_info);

    /// \brief Callback executed when the ball needs to be restarted.
    /// \param[in] _unused Unused parameter.
    private: void OnRestartBall(ConstIntPtr &/*_unused*/);

    /// \brief Callback executed when the game needs to be restarted.
    /// \param[in] _unused Unused parameter.
    private: void OnRestartGame(ConstIntPtr &/*_unused*/);

    /// \brief Set the current game state.
    /// \param[in] _newState New state to transition.
    public: void SetCurrentState(State<FoosballDemoPlugin> &_newState);

    /// \brief Seconds remaining to finish the game.
    public: common::Time gameTime;

    /// \brief Length of the table (m).
    public: float tableLength;

    /// \brief Height of the table (m).
    public: float tableHeight;

    /// \brief Player "A" score.
    public: int scoreA;

    /// \brief Player "B" score.
    public: int scoreB;

    /// \brief Pointer to the ball.
    public: physics::ModelPtr ball;

    /// \brief State to represent the kickoff period before the game starts.
    public: KickoffState kickoffState = {"kickoff", this};

    /// \brief State to represent a goal scored by team "A".
    public: GoalState goalAState = {"goalA", this, &this->scoreA};

    /// \brief State to represent a goal scored by team "B".
    public: GoalState goalBState = {"goalB", this, &this->scoreB};

    /// \brief State to represent the regular foosball play.
    public: PlayState playState = {"play", this};

    ///  \brief State to represent the end of the game.
    public: FinishedState finishedState = {"finished", this};

    /// \brief Default game time in seconds.
    private: int kDefaultGameTime = 180;

    /// \brief World pointer.
    private: physics::WorldPtr world;

    /// \brief SDF for this plugin.
    private: sdf::ElementPtr sdf;

    /// \brief Pointer to a node for communication.
    private: transport::NodePtr gzNode;

    /// \brief Time publisher.
    private: transport::PublisherPtr timePub;

    /// \brief Score publisher.
    private: transport::PublisherPtr scorePub;

    /// \brief Game state publisher.
    private: transport::PublisherPtr statePub;

    /// \brief "Restart ball" subscriber.
    private: transport::SubscriberPtr restartBallSub;

    /// \brief "Restart game" subscriber.
    private: transport::SubscriberPtr restartGameSub;

    /// \brief Pointer to the update event connection.
    private: event::ConnectionPtr updateConnection;

    /// \brief Sim time at which the game started.
    private: common::Time startTimeSim;

    /// \brief Game duration.
    private: common::Time gameDuration;

    /// \brief Current state.
    private: State<FoosballDemoPlugin> *currentState;

    /// \brief Mutex to avoid race conditions.
    private: std::mutex mutex;
  };
}
#endif
