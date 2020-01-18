/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#ifndef GAZEBO_VARIABLE_GEARBOX_PLUGIN_HH
#define GAZEBO_VARIABLE_GEARBOX_PLUGIN_HH

#include <memory>
#include <sdf/Element.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/util/system.hh>

namespace gazebo
{
  class VariableGearboxPluginPrivate;

  /// \brief A plugin that uses piecewise cubic Hermite splines to support
  /// arbitrary smooth input-output relationships between the input and output
  /// angles of a gearbox. It currently works for the ODE gearbox joint.
  ///
  /// The ODE gearbox joint now supports linear constriants of the form:
  /** \verbatim
      angle1 - refAngle1 = -ratio * (angle2 - refAngle2)
  \endverbatim */
  /// At each timestep, this plugin linearizes the splines based on the
  /// current joint state to compute refAngle1, refAngle2, and ratio
  /// and sets them in the ODE gearbox joint. The ODE solver then solves
  /// a linearized form of the nonlinear variable gearbox constraint.
  ///
  /// The plugin parameters are the gearbox_joint_name as a string and
  /// multiple x_y_dydx parameters for specifying points and slopes of
  /// the input-output curves that will be interpolated by splines:
  ///
  /// x: angle of the input joint
  /// y: angle of the output joint
  /// dy/dx: instantaneous gear ratio (change in output) / (change in input)
  ///
  /// The following example snippet is taken from variable_gearbox_plugin.world
  /** \verbatim
    <plugin name="variable_gearbox" filename="libVariableGearboxPlugin.so">
      <gearbox_joint_name>demo_joint_types::gearbox_demo</gearbox_joint_name>
      <x_y_dydx>1.2  -1.2  -1.0</x_y_dydx>
      <x_y_dydx>1.8  -7.5 -20.0</x_y_dydx>
    </plugin>
  \endverbatim */
  /// To aid in visualizing the splines produced by this plugin, please see
  /// the following ipython notebook:
  /// https://gist.github.com/scpeters/f7d87dd6578dee3fb95b90b402046679
  class GAZEBO_VISIBLE VariableGearboxPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: VariableGearboxPlugin();

    /// \brief Destructor
    public: virtual ~VariableGearboxPlugin();

    /// \brief Load the controller
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the gear ratios based on current joint position.
    /// \param[in] _info Update information provided by the server.
    private: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Private data pointer.
    private: std::unique_ptr<VariableGearboxPluginPrivate> dataPtr;
  };
}
#endif
