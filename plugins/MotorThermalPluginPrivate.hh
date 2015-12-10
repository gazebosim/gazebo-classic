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
#ifndef _GAZEBO_PLUGINS_MOTORTHERMALPLUGIN_PRIVATE_HH_
#define _GAZEBO_PLUGINS_MOTORTHERMALPLUGIN_PRIVATE_HH_

#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/common/CommonTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/transport/TransportTypes.hh>

namespace gazebo
{
  /// \internal
  /// \brief Private data for the motor thermal plugin
  class MotorThermalPluginPrivate
  {
    /// \brief Pointer to the world.
    public: physics::WorldPtr world;

    /// \brief Pointer to the joint.
    public: physics::JointPtr joint;

    /// \brief Connect to Gazebo's update event.
    public: event::ConnectionPtr updateConnection;

    /// \brief Electrical resistance, in ohms.
    public: double electricResistance = 1.16;

    /// \brief Inner thermal resistance
    /// In kelvins per watt
    public: double innerThermalResistance = 1.93;

    /// \brief Outer thermal resistance
    /// In kelvins per watt
    public: double outerThermalResistance = 4.65;

    /// \brief Thermal conductance of the coil.
    /// In watts per meter kelvin: W/(m K).
    public: double coilThermalConductance = 21.55;

    /// \brief Thermal conductance of the case.
    /// In watts per meter kelvin: W/(m K).
    public: double caseThermalConductance = 240.86;

    /// \brief Thermal conductance of the atmostphere.
    /// In watts per meter kelvin: W/(m K).
    public: double atomosphereThermalConductance = 1000.0;

    /// \brief Temperature of the atmosphere, in kelvin
    public: common::Temperature atomosphereTemperature = 300.0;

    /// \brief Temperature of the coil, in kelvin
    public: common::Temperature coilTemperature = 300.0;

    /// \brief Temperature of the case, in kelvin
    public: common::Temperature caseTemperature = 300.0;

    /// \brief Multiplier that converts torque to amperes
    public: double torqueToAmp = 0.1;

    /// \brief Hz update rate
    public: double updateRate = 10.0;

    /// \brief Torque value
    public: double torque = 0.0;

    /// \brief Time of the last update
    public: common::Time lastUpdateTime;

    /// \brief Transport node pointer
    public: transport::NodePtr node;

    /// \brief Publisher of torque values
    public: transport::PublisherPtr torquePub;

    /// \brief Publisher of coil temperature values
    public: transport::PublisherPtr coilPub;

    /// \brief Publisher of case temperature values
    public: transport::PublisherPtr casePub;
  };
}
#endif
