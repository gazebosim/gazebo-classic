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
#ifndef GAZEBO_PLUGINS_WHEELSLIPPLUGIN_HH_
#define GAZEBO_PLUGINS_WHEELSLIPPLUGIN_HH_

#include <map>
#include <memory>
#include <string>

#include <ignition/math/Vector3.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/util/system.hh>

namespace gazebo
{
  // Forward declare private data class
  class WheelSlipPluginPrivate;

  /// \brief A plugin that updates ODE wheel slip parameters based
  /// on linear wheel spin velocity (radius * spin rate).
  /// It currently assumes that the fdir1 friction parameter is set
  /// parallel to the joint axis (often [0 0 1]) and that the link
  /// origin is on the joint axis.
  /// The ODE slip parameter is documented as Force-Dependent Slip
  /// (slip1, slip2) in the ODE user guide:
  /// http://ode.org/ode-latest-userguide.html#sec_7_3_7
  /// and it has units of velocity / force (m / s / N),
  /// similar to the inverse of a viscous damping coefficient.
  /// The slip_compliance parameters specified in this plugin
  /// are unitless, representing the lateral or longitudinal slip ratio
  /// (see https://en.wikipedia.org/wiki/Slip_(vehicle_dynamics) )
  /// to tangential force ratio (tangential / normal force).
  /// Note that the maximum force ratio is the friction coefficient.
  /// At each time step, these compliance are multipled by
  /// the linear wheel spin velocity and divided by the wheel_normal_force
  /// parameter specified below in order to match the units of the ODE
  /// slip parameters.
  ///
  /// A graphical interpretation of these parameters is provided below
  /// for a positive value of slip compliance.
  /// The horizontal axis corresponds to the slip ratio at the wheel,
  /// and the vertical axis corresponds to the tangential force ratio
  /// (tangential / normal force).
  /// As wheel slip increases, the tangential force increases until
  /// it reaches the maximum set by the friction coefficient.
  /// The slip compliance corresponds to the inverse of the slope
  /// of the force before it reaches the maximum value.
  /// A slip compliance of 0 corresponds to a completely vertical
  /// portion of the plot below.
  /// As slip compliance increases, the slope decreases.
  ///
  /** \verbatim
        |                                            .
        |      _________ friction coefficient        .
        |     /                                      .
        |    /|                                      .
        |   /-┘ slope is inverse of                  .
        |  /    slip compliance                      .
        | /                                          .
        |/                                           .
      --+-------------------------- slipRatio
        |

    <plugin filename="libWheelSlipPlugin.so" name="wheel_slip">
      <wheel link_name="wheel_front_left">
        <slip_compliance_lateral>0</slip_compliance_lateral>
        <slip_compliance_longitudinal>0.1</slip_compliance_longitudinal>
        <wheel_normal_force>100</wheel_normal_force>
      </wheel>
      <wheel link_name="wheel_front_right">
        <slip_compliance_lateral>0</slip_compliance_lateral>
        <slip_compliance_longitudinal>0.1</slip_compliance_longitudinal>
        <wheel_normal_force>100</wheel_normal_force>
      </wheel>
      <wheel link_name="wheel_rear_left">
        <slip_compliance_lateral>0</slip_compliance_lateral>
        <slip_compliance_longitudinal>0.1</slip_compliance_longitudinal>
        <wheel_normal_force>80</wheel_normal_force>
      </wheel>
      <wheel link_name="wheel_rear_right">
        <slip_compliance_lateral>0</slip_compliance_lateral>
        <slip_compliance_longitudinal>0.1</slip_compliance_longitudinal>
        <wheel_normal_force>80</wheel_normal_force>
      </wheel>
    </plugin>
   \endverbatim */
  class GAZEBO_VISIBLE WheelSlipPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: WheelSlipPlugin();

    /// \brief Destructor.
    public: virtual ~WheelSlipPlugin();

    // Documentation inherited
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation inherited
    public: virtual void Fini();

    /// \brief Get parent model.
    /// \return pointer to parent model.
    public: physics::ModelPtr GetParentModel() const;

    /// \brief Get wheel slip measurements.
    /// \param[out] _out Map of wheel name to a Vector3 of slip velocities.
    /// The Vector3.X value is the longitudinal slip in m/s,
    /// the Vector3.Y value is the lateral slip in m/s, and
    /// the Vector3.Z value is the product of radius and spin rate in m/s.
    public: void GetSlips(std::map<std::string, ignition::math::Vector3d> &_out)
            const;

    /// \brief Set unitless lateral slip compliance for all wheels.
    /// \param[in] _compliance unitless slip compliance to set.
    public: void SetSlipComplianceLateral(const double _compliance);

    /// \brief Set unitless longitudinal slip compliance for all wheels.
    /// \param[in] _compliance unitless slip compliance to set.
    public: void SetSlipComplianceLongitudinal(const double _compliance);

    /// \brief Transport callback for setting lateral slip compliance.
    /// \param[in] _msg Slip compliance encoded as string.
    private: void OnLateralCompliance(ConstGzStringPtr &_msg);

    /// \brief Transport callback for setting longitudinal slip compliance.
    /// \param[in] _msg Slip compliance encoded as string.
    private: void OnLongitudinalCompliance(ConstGzStringPtr &_msg);

    /// \brief Update the plugin. This is updated every iteration of
    /// simulation.
    private: void Update();

    /// \brief Private data pointer.
    private: std::unique_ptr<WheelSlipPluginPrivate> dataPtr;
  };
}
#endif
