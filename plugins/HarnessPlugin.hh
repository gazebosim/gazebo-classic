/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_PLUGINS_HARNESSPLUGIN_HH_
#define GAZEBO_PLUGINS_HARNESSPLUGIN_HH_

#include <string>
#include <vector>

#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/common/PID.hh"
#include "gazebo/common/Plugin.hh"

namespace gazebo
{
  /// \brief This plugin is designed to lower a model at a controlled rate.
  /// Joints between a harness model and a model to lower are created
  /// according to SDF provided to this plugin.
  ///
  /// A winch joint and detach joint can be specified. The winch joint,
  /// ideally a prismatic joint, has a PID controller. The detach joint,
  /// which can be the same joint as the winch joint, is detached on a given
  /// signal.
  ///
  /// Two topics are created:
  ///
  ///  1. ~/<plugin_model_name>/harness/velocity
  ///      - Message Type: GzString, expected to be a float
  ///      - Purpose: Set target winch velocity
  ///
  ///  2. ~/<plugin_model_name>/harness/detach
  ///      - Message Type: GzString, expected to be a bool ("true")
  ///      - Purpose: Detach the <detach> joint.
  ///
  /// For an example refer to:
  ///   - World file: worlds/harness.world
  ///   - Code: examples/stand_alone/harness
  class GAZEBO_VISIBLE HarnessPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: HarnessPlugin();

    /// \brief Destructor.
    public: virtual ~HarnessPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation Inherited.
    public: virtual void Init();

    /// \brief Set the target winch velocity
    /// \param[in] _value Target winch velocity.
    public: void SetWinchVelocity(const float _value);

    /// \brief Get the current winch velocity
    /// \return Velocity of the winch joint
    public: double WinchVelocity() const;

    /// \brief Detach the <detach> joint. Once the joint is detached, it
    /// cannot be reattached.
    public: void Detach();

    /// \brief Callback for World Update events.
    /// \param[in] _info Update information
    private: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Velocity control callback.
    /// \param[in] _msg Message data, interpretted as a float
    private: void OnVelocity(ConstGzStringPtr &_msg);

    /// \brief Detach control callback.
    /// \param[in] _msg Message data, interpretted as a bool
    private: void OnDetach(ConstGzStringPtr &_msg);

    /// \brief Get the index of a joint with the given name.
    /// \param[in] _name Name of the joint to find.
    /// \return Index into this->jointsto
    private: int JointIndex(const std::string &_name) const;

    /// \brief Vector of joints
    private: std::vector<physics::JointPtr> joints;

    /// \brief Index into the joints vector that specifies the winch joint.
    private: int winchIndex = 0;

    /// \brief Index into the joints vector that specifies the joint to detach.
    private: int detachIndex = 0;

    /// \brief Position PID controller for the winch
    private: common::PID winchPosPID;

    /// \brief Velocity PID controller for the winch
    private: common::PID winchVelPID;

    /// \brief Target winch position
    private: float winchTargetPos = 0.0;

    /// \brief Target winch velocity
    private: float winchTargetVel = 0.0;

    /// \brief Previous simulation time
    private: common::Time prevSimTime = common::Time::Zero;

    /// \brief Communication node
    /// \todo: Transition to ignition-transport in gazebo8
    private: transport::NodePtr node;

    /// \brief Velocity control subscriber
    /// \todo: Transition to ignition-transport in gazebo8
    private: transport::SubscriberPtr velocitySub;

    /// \brief Detach control subscriber
    /// \todo: Transition to ignition-transport in gazebo8
    private: transport::SubscriberPtr detachSub;

    /// \brief Connection to World Update events.
    private: event::ConnectionPtr updateConnection;
  };
}
#endif
