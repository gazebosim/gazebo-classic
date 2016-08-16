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
#ifndef GAZEBO_PLUGINS_RT_CONTROL_SYNCHRONIZATION_PLUGIN_HH_
#define GAZEBO_PLUGINS_RT_CONTROL_SYNCHRONIZATION_PLUGIN_HH_

#include <sdf/sdf.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

namespace gazebo
{
  // Forward declare private data class
  class RTControlSynchronizationPluginPrivate;

  /// \brief A RealTime Controller Synchronization Example
  /// modeled after DRCSim AtlasPlugin
  ///
  /// The plugin requires the following parameters:
  /// <controller_topic>    controller is on this topic
  ///
  /// The following parameters are optional:
  ///
  class GAZEBO_VISIBLE RTControlSynchronizationPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: RTControlSynchronizationPlugin();

    /// \brief Destructor.
    public: ~RTControlSynchronizationPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Bypass synchronization and step simulation once.
    public: void Tic();

    /// \brief Update the control surfaces controllers.
    /// \param[in] _info Update information provided by the server.
    protected: virtual void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Update PID Joint controllers.
    /// \param[in] _dt time step size since last update.
    protected: virtual void ApplyRobotCommandToSim(const double _dt);

    /// \brief Receive robot commands from controller
    protected: virtual void ReceiveRobotCommand();

    /// \brief Receive robot commands from controller
    protected: void RobotCommandIn(ConstAnyPtr &_msg);

    /// \brief Send robot state to controller
    protected: void RobotStateOut();

    /// \brief Send robot state to controller
    protected: virtual void SendRobotState();

    /// \brief Expected update interval.
    /// If the age of the current command is older than this interval
    /// than we will pause simulation to wait for external
    /// controller to send a new command.
    /// For example, if the external real-time controller is running
    /// at 1kHz, then simulation should expect a new command within
    /// 1 millisecond of the last received command, i.e. command age
    /// should be less than 1 millisecon.
    private: double timeoutMs;

    /// \brief Private data pointer.
    private: std::unique_ptr<RTControlSynchronizationPluginPrivate> dataPtr;

    /// \brief Node used for using Gazebo communications.
    private: transport::NodePtr node;

    /// \brief Subscriber pointer.
    private: transport::SubscriberPtr controlSub;

    /// \brief Publisher pointer.
    private: transport::PublisherPtr statePub;
  };
}
#endif
