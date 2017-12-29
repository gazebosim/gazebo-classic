/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#ifndef GAZEBO_PLUGINS_JOINTCONTROLPLUGIN_HH_
#define GAZEBO_PLUGINS_JOINTCONTROLPLUGIN_HH_

#include <memory>

#include <sdf/sdf.hh>

#include <gazebo/common/Plugin.hh>

namespace gazebo
{
  /// \brief Plugin that initializes joint controllers.
  ///
  /// \verbatim
  ///    <!-- Initialize the joint control parameters for a model. -->
  ///    <plugin name="init_joint_control" filename="libJointControlPlugin.so">
  ///
  ///      <!-- Set up a PID position controller. -->
  ///      <!-- Valid controller types: "force", "position", or "velocity". -->
  ///      <controller type="position">
  ///
  ///        <!-- Name of a joint for which to apply this controller. -->
  ///        <joint>joint_0</joint>
  ///        <!-- Multiple joints may be specified. -->
  ///        <joint>joint_1</joint>
  ///        <!-- C++ regular expressions may be used. -->
  ///        <joint>joint_(.*)</joint>
  ///
  ///        <!-- Joint controller effort (force or torque),
  ///             target position (distance or radians), or
  ///             target velocity(distance/second or radians/second). -->
  ///        <target>1.0</target>
  ///
  ///        <!-- PID controller gains (for position or velocity control). -->
  ///        <pid_gains>1 0.1 0.01</pid_gains>
  ///      </controller>
  ///
  ///      <!-- Multiple controllers may be specified... -->
  ///
  ///    </plugin>
  /// \endverbatim
  ///
  /// See worlds/init_joint_control.world for a complete example.
  class GAZEBO_VISIBLE JointControlPlugin : public ModelPlugin
  {
    /// \brief Parses plugin parameters and sends them to the joint controller.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  };
}
#endif
