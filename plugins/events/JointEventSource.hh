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

#ifndef _GAZEBO_PLUGINS_EVENTS_JOINTEVENTSOURCE_HH_
#define _GAZEBO_PLUGINS_EVENTS_JOINTEVENTSOURCE_HH_

#include <string>

#include "plugins/events/EventSource.hh"

namespace gazebo
{
  /// \brief The event generator class. Events are generated when joint enters
  /// or leaves a certain trigger state. This type of event works with joints
  /// that have a single axis (revolute or prismatic). These are the most
  /// common for actuated joints.
  /// Triggers must be defined in the world, but models can be created during
  /// the simulation. Triggers cannot overlap.
  ///
  ///
  /// \verbatim
  ///
  ///  This is an example joint event. It is triggered when the joint named
  ///  "joint" in the model "revoluter" has an angle value that enters or
  ///  leaves the range [3, 3.1416]. Triggers can also depend on the position,
  ///  velocity or applied force.
  ///
  ///  <event>
  ///    <name>joint_angle</name>
  ///    <type>joint</type>
  ///    <model>revoluter</model>
  ///    <joint>joint</joint>
  ///    <range>
  ///      <type>normalized_angle</type>
  ///      <min>3</min>
  ///      <max>3.1416</max>
  ///    </range>
  ///  </event>
  ///
  /// \endverbatim
  class JointEventSource: public EventSource
  {
    /// \enum Range
    /// \brief The type of data range measured
    public: enum Range {
                /// \brief Absolute position (or angle, for revolute joints)
                POSITION,
                /// \brief Normalized angle (between -PI and PI)
                ANGLE,
                /// \brief Velocity or angular velocity
                VELOCITY,
                /// \brief Applied force (or torque, for revolute joints)
                FORCE,
                /// \brief invalid
                INVALID
              };


    /// \brief Constructor
    /// \param[in] _pub the publisher for the SimEvents
    /// \param[in] _world Pointer to the world.
    public: JointEventSource(transport::PublisherPtr _pub,
                physics::WorldPtr _world);

    /// \brief Initialize the event
    public: virtual void Init();

    /// \brief Called every simulation step
    public: void Update();

    /// \brief Prints data about the event source to the log (useful for debug)
    public: void Info() const;

    /// \brief Loads the full name of the model and the triggers from the world
    /// file.
    /// \param[in] _sdf The root sdf element for this joint event
    public: virtual void Load(const sdf::ElementPtr _sdf);

    /// \brief Looks for the model and the joint
    /// \return true if found
    private: bool LookupJoint();

    /// \brief Utility range to string conversion
    /// returns The current range as a string
    private: std::string RangeAsString() const;

    /// \brief Sets the range type from a string
    /// \param[in] _rangeStr the range. Possible values are: "velocity",
    /// "position", "normalized_angle" or "applied_force"
    private: void SetRangeFromString(const std::string &_rangeStr);

    /// \brief Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    /// \brief The model used for the in trigger check.
    private: std::string modelName;

    /// \brief The joint name
    private: std::string jointName;

    /// \brief A pointer to the model
    private: physics::ModelPtr model;

    /// \brief A pointer to the Joint
    private: physics::JointPtr joint;

    /// \brief Minimum joint value (the type is determined by range member)
    private: double min;

    /// \brief Maximum joint value
    private: double max;

    /// \brief The type of data in the range (see Range enum)
    private: Range range;

    /// \brief True when the joint is currently inside the trigger condition
    private: bool isTriggered;
  };
}

#endif
