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
#ifndef _GAZEBO_PLUGINS_MOTORTHERMALPLUGIN_HH_
#define _GAZEBO_PLUGINS_MOTORTHERMALPLUGIN_HH_

#include <memory>
#include <sdf/sdf.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/commom/Plugin.hh>
#include <gazebo/util/system.hh>

namespace gazebo
{
  // Forward declare private data class
  class MotorThermalPluginPrivate;

  /// \brief This plugin simulates and computes the thermal propagation of
  /// motor in accordance with the following equations:
  ///
  /// *  atmosphere: dTo = (Tc-To)/Ro
  /// *  motor case: dTc = (To-Tc)/Ro + (Ti-Tc)/Ri
  /// *  motor coil: dTi = (Ti-Tc)/Ri + I * E
  ///
  /// where, C=thermal conductance, R=thermal resistance,
  ///
  /// * Tc = case temperature.
  /// * To = outer temperature.
  /// * Ti = coil temperature.
  /// * Ro = outer thermal resistance
  /// * Ti = coil temperature.
  /// * Ri = inner thermal resistance
  /// * I = electric current,
  /// * E = electric resistance,
  ///
  /// Example:
  /// <plugin filename="libMotorThermalPlugin.so" name="motor_thermal">
  ///
  ///   <!-- Hz rate for this plugin -->
  ///   <update_rate></update_rate>
  ///
  ///   <!-- Name of the joint to which the motor is attached -->
  ///   <joint>joint_1</joint>
  ///
  ///   <!-- Electric resistance of the motor, in Ohms -->
  ///   <electric_resistance>1.16</electric_resistance>
  ///
  ///   <!-- This value is multiplied by the motor torque to arrive at
  ///        the electric current, in Amperes -->
  ///   <torque_to_amp></torque_to_amp>
  ///
  ///   <!-- Inner thermal resistance in kelvins per watt -->
  ///   <inner_thermal_resistance></inner_thermal_resistance>
  ///
  ///   <!-- Outer thermal resistance in kelvins per watt -->
  ///   <outer_thermal_resistance></outer_thermal_resistance>
  ///
  ///   <!-- Thermal conductance of the coil, in watts per
  ///   meter kelvin: W/(m K) -->
  ///   <coil_thermal_conductance></coil_thermal_conductance>
  ///
  ///   <!-- Thermal conductance of the case, in watts per
  ///   meter kelvin: W/(m K) -->
  ///   <case_thermal_conductance></case_thermal_conductance>
  ///
  ///   <!-- Thermal conductance of the atmosphere, in watts per
  ///   meter kelvin: W/(m K) -->
  ///   <atomosphere_thermal_conductance></atomosphere_thermal_conductance>
  ///
  ///   <!-- Temperature of the atmosphere, in kelvin -->
  ///   <atomosphere_temperature>300.0</atomosphere_temperature>
  ///
  ///   <!-- Temperature of the case, in kelvin -->
  ///   <case_temperature>300.0</case_temperature>
  ///
  ///   <!-- Temperature of the coil, in kelvin -->
  ///   <coil_temperature>300.0</coil_temperature>
  ///
  /// </plugin>
  class GAZEBO_VISIBLE MotorThermalPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: MotorThermalPlugin();

    /// \brief Destructor
    public: virtual ~MotorThermalPlugin() = default;

    /// \brief Load the plugin
    /// \param[in] _model Pointer to the model
    /// \param[in] _sdf Pointer to the SDF parameters.
    public: virtual Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Get the torque value.
    /// \return Joint torque
    public: float Torque() const;

    /// \brief Get the temperature of the motor case.
    /// \return Motor case temperature.
    public: Temperature CaseTemperature() const;

    /// \brief Get the temperature of the motor coil.
    /// \return Motor coil temperature.
    public: Temperature CoilTemperature() const;

    /// \brief Handle simulation update calls
    private: void OnUpdate();

    /// \internal
    /// \brief Private data pointer
    private: std::unique_ptr<MotorThermalPluginPrivate> dataPtr;
  };
}
#endif
