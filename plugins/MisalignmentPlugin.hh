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

#ifndef GAZEZBO_PLUGIN_MISALIGNMENT_PLUGIN_HH_
#define GAZEZBO_PLUGIN_MISALIGNMENT_PLUGIN_HH_

#include <memory>

#include "gazebo/common/Plugin.hh"

namespace gazebo
{
  class MisalignmentPluginPrivate;

  /// \brief Plugin which emits gazebo transport message indicating the
  /// alignment of two poses.
  ///
  /// Example usage:
  ///
  ///  <plugin name="misalignment_plugin" filename="libMisalignmentPlugin.so">
  ///    <!-- Misalignment will be given as the pose of the second <pose> in
  ///         a frame defined by the first <pose>. The poses can be given in a
  ///         frame defined by the pose of an entity (model or link) named in
  ///         the attribute 'frame'. If no 'frame' is given then the pose is
  ///         in world frame. -->
  ///    <pose frame="robot::arm_link">0 .2 0 1.570796 0 0</pose>
  ///    <!-- Must have two <pose> -->
  ///    <pose frame="coffee::handle_link">0 0 0 0 0 0</pose>
  ///
  ///    <!-- Namespace for gazebo transport topics
  ///           /<namespace>/misalignment
  ///           If <debug> is True
  ///           /<namespace>/debug/*
  ///    -->
  ///    <namespace>gazebo/robot</namespace>
  ///
  ///    <!-- If true then debug info will be published -->
  ///    <debug>true</debug>
  ///
  ///    <!-- True to enable automatically, false so it must be enabled
  ///         via a message - true by default -->
  ///    <enabled>true</enabled>
  ///
  ///  </plugin>
  ///
  class GAZEBO_VISIBLE MisalignmentPlugin : public WorldPlugin
  {
    // Documentation inherited
    public: MisalignmentPlugin();

    // Documentation inherited
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override;

    /// \brief Pointer to private data
    private: std::unique_ptr<MisalignmentPluginPrivate> dataPtr;
  };
}

#endif
