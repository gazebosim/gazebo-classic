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
#ifndef _GAZEBO_LINEAR_BATTERY_PLUGIN_HH_
#define _GAZEBO_LINEAR_BATTERY_PLUGIN_HH_

#include <string>
#include <map>

#include "gazebo/common/Plugin.hh"
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/physics/physics.hh"

namespace gazebo
{
  /// \brief A plugin that simulates a linear battery.
  class GAZEBO_VISIBLE LinearBatteryPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: LinearBatteryPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation Inherited.
    public: virtual void Init();

    // Documentation Inherited.
    public: virtual void Reset();

    /// \brief Callback for Battery Update events.
    protected: virtual double OnUpdateVoltage(double _voltage,
                   const common::Battery::PowerLoad_M &_powerLoads);

    /// \brief Connection to World Update events.
    protected: event::ConnectionPtr updateConnection;

    /// \brief Pointer to world.
    protected: physics::WorldPtr world;

    /// \brief Pointer to physics engine.
    protected: physics::PhysicsEnginePtr physics;

    /// \brief Pointer to model containing plugin.
    protected: physics::ModelPtr model;

    /// \brief Pointer to link containing battery link.
    protected: physics::LinkPtr link;

    /// \brief Pointer to battery contained in link.
    protected: common::BatteryPtr battery;

    /// \brief SDF for this plugin;
    protected: sdf::ElementPtr sdf;

    /// \brief Open-circuit voltage.
    /// E(t) = e0 + e1 * Q(t) / c
    protected: double e0;
    protected: double e1;

    /// \brief Initial battery charge in Ah.
    protected: double q0;

    /// \brief Battery capacity in Ah.
    protected: double c;

    /// \brief Battery inner resistance in Ohm.
    protected: double r;

    /// \brief Current low-pass filter characteristic time in seconds.
    protected: double tau;

    /// \brief Raw battery current in A.
    protected: double iraw;

    /// \brief Smoothed battery current in A.
    protected: double ismooth;

    /// \brief Instantaneous battery charge in Ah.
    protected: double q;
  };
}
#endif
