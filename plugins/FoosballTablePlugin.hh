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

#ifndef _GAZEBO_FOOSBALLTABLE_PLUGIN_HH_
#define _GAZEBO_FOOSBALLTABLE_PLUGIN_HH_

#include <array>
#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <sdf/sdf.hh>
#include "gazebo/common/PID.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/UpdateInfo.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  /// \class FoosballPlayer FoosballTablePlugin.hh
  /// \brief A class that moves a set of rods of the table based on a Hydra
  /// device controlled by a player.
  class GAZEBO_VISIBLE FoosballPlayer
  {
    /// \def Rod_t
    /// \brief A rod is composed by two joints (prismatic and revolute).
    using Rod_t = std::array<physics::JointPtr, 2>;

    /// \def Rod_V
    /// \brief A vector of rods.
    using Rod_V = std::vector<Rod_t>;

    /// \def Hydra_t
    /// \brief A Hydra is composed by two controllers (left and right).
    /// The key will be "left_controller" or "right_controller".
    /// The value represents the index of the rod controlled by the controller.
    using Hydra_t = std::map<std::string, unsigned int>;

    /// \brief Constructor.
    /// \brief \param[in] _hydraTopic Topic name in which the Hydra associated
    /// to this player will publish updates. E.g.: ~/hydra0
    public: FoosballPlayer(const std::string &_hydraTopic);

    /// \brief Load a <player> section of the plugin SDF.
    /// \param [in] _model Pointer to the model.
    /// \param [in] _sdf Pointer to the <player> section of the plugin SDF.
    /// \return true on success or false otherwise.
    public: bool Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Update the rods controlled by this player.
    public: void Update();

    /// \brief Callback executed every time we receive a new update from the
    /// Hydra bound to this player.
    /// \param[in] _msg The hydra message.
    private: void OnHydra(ConstHydraPtr &_msg);

    /// \brief Check if we have to change the active rods for this player.
    private: void UpdateActiveRod();

    /// \brief Each left/right Hydra controller controls a set of rods but
    /// only one rod at a time. This method switches the rod to control.
    /// The rods change in a circular way every time the trigger button is
    /// pressed.
    /// \param[in] _side "left_controller" or "right_controller".
    private: void SwitchRod(const double _leftDir, const double _rightDir);

    private: void PublishVisualMsg(std::string &_name, std::string &_parentName,
        std::string &_color);

    /// \brief Num of rods on the table per team.
    private: unsigned int kNumRodsPerTeam = 4;

    /// \brief Pointer to the model;
    private: physics::ModelPtr model;

    private: std::string team;

    /// \brief Pointer to a node for communication.
    private: transport::NodePtr gzNode;

    /// \brief "Restart ball" publisher.
    private: transport::PublisherPtr restartBallPub;

    /// \brief "Restart ball" publisher.
    private: transport::PublisherPtr visualPub;

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

    /// \brief Last Hydra message received.
    private: boost::shared_ptr<const gazebo::msgs::Hydra> hydraMsgPtr;

    /// \brief Hydra controller used by this player.
    private: Hydra_t hydra;

    /// \brief The vector of rods controlled by this player.
    private: Rod_V rods;

    /// \brief The vector of rods controlled by this player.
    private: std::vector<std::string> shafts;

    /// \brief Stores the last known X position and roll angle.
    /// of the left/right controller. Those parameters are the ones we use in
    /// the plugin to compute the controller velocity.
    private: std::map<std::string, std::array<double, 2>> lastHydraPose;

    /// \brief Stores the last positions of the active rods.
    /// If we switch rods we have to modify the position of the new active rod
    /// to match the position of the previous one.
    private: std::map<std::string, std::array<math::Angle, 2>> lastRodPose;

    /// \brief Last time that the plugin was updated.
    private: common::Time lastUpdateTime;

    /// \brief Did the user move the joystick to change the rods?
    private: bool pendingRodChange = false;

    /// \brief Value of the left joystick during a rod transition.
    private: double leftJoyCmd = 0.0;

    /// \brief Value of the right joystick during a rod transition.
    private: double rightJoyCmd = 0.0;

    /// \brief This multiplier will be used to invert the velocity applied to
    /// a rod depending on the player's viewpoint.
    private: double invert = 1.0;

    /// \brief We have to restart the ball position.
    private: bool restartBallPending = false;
  };

  /// \class FoosballTablePlugin FoosballTablePlugin.hh
  /// \brief Class that moves the foosball table rods according to the movements
  /// of some Hydra controllers. Each controller will be bound to a set of rods.
  ///
  /// The plugin accepts 'n' blocks of <player> elements as follows:
  /// <player>
  ///   <team>blue</team>
  ///   <invert_control>0</invert_control>
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
  /// <invert_control> is an optional tag. By default, the plugin assumes
  /// that the players' viewpoint matches his/her side of the table.
  /// When <invert_control> is 1, the plugin will assume that the player is
  /// playing from the other side of the table and the controllers are inverted.
  /// <left_controller> and <right_controller> are optional. If present,
  /// each tag associates a set of rods to the left or right controller.
  /// The control of a rod will change by pressing the 'trigger' button.
  /// <rod> is a required tag and specifies a rod number.
  class GAZEBO_VISIBLE FoosballTablePlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: FoosballTablePlugin() = default;

    /// \brief Destructor.
    public: ~FoosballTablePlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Update the foosball table rods.
    /// \param[in] _info Update information provided by the server.
    private: void Update(const common::UpdateInfo &_info);

    /// \brief Callback executed when the ball needs to be restarted.
    /// \param[in] _unused Unused parameter.
    private: void OnShakeTable(ConstIntPtr &/*_unused*/);

    /// \brief Pointer to the update event connection.
    private: event::ConnectionPtr updateConnection;

    /// \brief Vector of players that will control the foosball rods.
    private: std::vector<std::unique_ptr<FoosballPlayer>> players;

    /// \brief Node used for using Gazebo communications.
    private: transport::NodePtr node;

    /// \brief Subscriber pointer.
    private: transport::SubscriberPtr shakeTableSub;

    /// \brief Pointer to the model;
    private: physics::ModelPtr model;
  };
}
#endif
