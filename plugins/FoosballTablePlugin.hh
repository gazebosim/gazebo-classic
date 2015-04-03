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

#ifndef _GAZEBO_FOOSBALL_PLUGIN_HH_
#define _GAZEBO_FOOSBALL_PLUGIN_HH_

#include <list>
#include <map>
#include <mutex>
#include <sdf/sdf.hh>
#include "gazebo/common/UpdateInfo.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  /// \def Rod_t
  /// \brief A rod is composed by two joints (prismatic and revolute).
  typedef std::array<physics::JointPtr, 2> Rod_t;
  typedef std::map<std::string, std::vector<Rod_t>> Controller_t;

  class GAZEBO_VISIBLE FoosballPlayer
  {
    /// \brief Constructor.
    public: FoosballPlayer(const std::string &_hydraTopic);

    /// \brief Load.
    public: bool Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Update the rods controlled by this player.
    public: void Update();

    /// \brief Callback executed every time a new hydra message is received.
    /// \param[in] _msg The hydra message.
    private: void OnHydra(ConstHydraPtr &_msg);

    /// \brief Initialize the starting position of the Hydra controllers.
    private: void Restart();

    /// \brief Each Hydra controller controls one rod. This method switches the
    /// rod to control. The rods change in a circular way every time the trigger
    /// is pressed.
    private: void SwitchRod(std::vector<Rod_t> &_aController);

    /// \brief Pointer to the update event connection.
    private: event::ConnectionPtr updateConnection;

    /// \brief Node used for using Gazebo communications.
    private: transport::NodePtr node;

    /// \brief Subscriber pointer.
    private: transport::SubscriberPtr hydraSub;

    /// \brief Reset pose of the left Hydra controller.
    private: math::Pose resetPoseLeft;

    /// \brief Reset pose of the right Hydra controller.
    private: math::Pose resetPoseRight;

    /// \brief Is Hydra control activated?
    private: bool activated = false;

    /// \brief Mutex to protect the hydra messages.
    private: std::mutex msgMutex;

    /// \brief Topic in which the Hydra updates for this player are published.
    private: std::string hydraTopic;

    /// \brief Hydra messages.
    private: std::list<boost::shared_ptr<msgs::Hydra const>> hydraMsgs;

    /// \brief
    private: Controller_t controller;

    private: math::Pose basePoseRight;
    private: math::Pose basePoseLeft;
    private: math::Pose leftStartPose;
    private: math::Pose rightStartPose;

    private: bool leftTriggerPressed = false;
    private: bool rightTriggerPressed = false;
  };

  /// \class FoosballTablePlugin FoosballTablePlugin.hh
  /// \brief Class that moves the foosball table rods according to the movements
  /// of some Hydra controllers. Each controller will be bound to a set of rods.
  ///
  /// The plugin accepts 'n' blocks of <player> elements as follows:
  /// <player>
  ///   <team>blue</team>
  ///   <left_controller>
  ///     <rod>0</rod>
  ///     <rod>1</rod>
  ///   </left_controller>
  ///   <right_controller>
  ///     <rod>2</rod>
  ///     <rod>3</rod>
  ///   </right_controller>
  /// </player>
  ///
  /// <team> is a required tag and should contain 'red' or 'blue'.
  /// <left_controller> and <right_controller> are optional. If present,
  /// each tag associates a set of rods to the left or right controller.
  /// The control of a rod will change by pressing the 'trigger' button.
  /// <rod> is a required tag and specifies a rod number.
  class GAZEBO_VISIBLE FoosballTablePlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: FoosballTablePlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Update the foosball table rods.
    /// \param[in] _info Update information provided by the server.
    private: void Update(const common::UpdateInfo &_info);

    /// \brief Pointer to the update event connection.
    private: event::ConnectionPtr updateConnection;

    /// \brief Vector of players that will control the foosball rods.
    private: std::vector<std::unique_ptr<FoosballPlayer>> players;
  };
}
#endif
