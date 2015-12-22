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

#ifndef _GAZEBO_ELEVATOR_PLUGIN_HH_
#define _GAZEBO_ELEVATOR_PLUGIN_HH_

#include <string>

#include <sdf/sdf.hh>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/util/system.hh>

namespace gazebo
{
  /// Forward declare private data.
  class ElevatorPluginPrivate;

  /// \brief Plugin to control a elevator. This plugin will listen for
  /// door and lift events on a specified topic.
  ///
  /// \verbatim
  ///   <plugin filename="libElevatorPlugin.so" name="elevator_plugin">
  ///     <lift_joint>elevator::lift</lift_joint>
  ///     <door_joint>elevator::door</door_joint>
  ///     <floor_height>3.075</floor_height>
  ///
  ///     <!-- Time the elevator door will stay open in seconds -->
  ///     <door_wait_time>5</door_wait_time>
  ///
  ///     <topic>~/elevator</topic>
  ///   </plugin>
  /// \endverbatim
  ///
  /// See worlds/elevator.world for a complete example.
  class GAZEBO_VISIBLE ElevatorPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: ElevatorPlugin();

    /// \brief Destructor.
    public: ~ElevatorPlugin();

    // Documentation inherited
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation inherited
    public: virtual void Reset();

    /// \brief Move to a particular floor.
    /// \param[in] _floor Number of the floor to move the elevator to.
    public: void MoveToFloor(const int _floor);

    /// \brief Update the plugin once every iteration of simulation.
    /// \param[in] _info Update information provided by the server.
    private: void Update(const common::UpdateInfo &_info);

    /// \brief Receives messages on the elevator's topic.
    /// \param[in] _msg The string message that contains a command.
    private: void OnElevator(ConstGzStringPtr &_msg);

    /// \brief Private data pointer
    private: ElevatorPluginPrivate *dataPtr;
  };
}
#endif
