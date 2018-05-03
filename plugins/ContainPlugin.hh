/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#ifndef GAZEBO_PLUGINS_CONTAINPLUGIN_HH_
#define GAZEBO_PLUGINS_CONTAINPLUGIN_HH_

#include <memory>

#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"

namespace gazebo
{
  class ContainPluginPrivate;

  /// \brief Plugin which emits Ignition Transport messages according to whether
  /// an entity's origin is inside or outside a given volume. A message is only
  /// published when the state changes.
  ///
  /// Example usage:
  ///
  ///  <plugin name="containRobotArm" filename="libContainPlugin.so">
  ///
  ///    <!-- True to enable automatically, false so it must be enabled
  ///         via a message - true by default -->
  ///    <enabled>true</enabled>
  ///
  ///    <!-- Scoped name of entity to check -->
  ///    <entity>robot::arm_link</entity>
  ///
  ///    <!-- Namespace for Ignition Transport topic and service:
  ///           /<namespace>/box/contains : topic where true / false messages
  ///                                       are published.
  ///           /<namespace>/box/enable : service to turn plugin on and off.
  ///    -->
  ///    <namespace>gazebo/robot</namespace>
  ///
  ///    <!-- Pose of the geometry's center point in world coordinates -->
  ///    <pose>10 10 2 0 0 1.57</pose>
  ///
  ///    <!-- OR Pose of the geometry's center in link frame coordinates -->
  ///    <pose frame="model_name::link_name">10 10 2 0 0 1.57</pose>
  ///
  ///    <!-- Geometry of the volume to check, uses the SDF geometry spec:
  ///         http://sdformat.org/spec?ver=1.6&elem=geometry
  ///         currently <box> is supported -->
  ///    <geometry>
  ///      <box>
  ///        <size>1 1 4</size>
  ///      </box>
  ///    </geometry>
  ///
  ///  </plugin>
  ///
  class GAZEBO_VISIBLE ContainPlugin : public WorldPlugin
  {
    // Documentation inherited
    public: ContainPlugin();

    // Documentation inherited
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override;

    /// \brief Called every world iteration on world update begin.
    /// \param[in] _info Update info.
    private: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Enables or disables the plugin.
    /// \param[in] _enable False to disable and true to enable the plugin.
    /// \return True when the operation succeed or false otherwise
    /// (e.g.: trying to enable the plugin when it is already enabled).
    private: bool Enable(const bool _enable);

    /// \brief Callback for enable service using Ignition messages.
    /// \param[in] _req Request, true to enable
    /// \param[out] _res Response message indicating success or failure.
    /// \return True if successful.
    // cppcheck-suppress unusedPrivateFunction
    private: bool EnableIgn(const ignition::msgs::Boolean &_req,
                            ignition::msgs::Boolean &_res);

    /// \brief Publish a message indicating if the box contains an entity
    /// \param[in] _contains True if the geometry contains the target
    private: void PublishContains(bool _contains);

    /// \brief Pointer to private data
    private: std::unique_ptr<ContainPluginPrivate> dataPtr;
  };
}

#endif
