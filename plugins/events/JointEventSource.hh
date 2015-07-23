/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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
  /// or leaves a certain trigger state.
  /// Triggers must be defined in the world, but models can be created during
  /// the simulation. Triggers cannot overlap.
  class JointEventSource: public EventSource
  {
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
    /// \param[in] _sdf
    public: virtual void Load(const sdf::ElementPtr _sdf);

    /// \brief Looks for the model and the joint
    /// \return true if found
    private: bool LookupJoint();

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

    /// \brief flag  that enables position range events
    private: bool positionRange;

    /// \brief minimum joint position.
    private: double minPosition;

    /// \brief maximum joint position
    private: double maxPosition;

    /// \brief true if the joint is currently inside the trigger condition
    private: bool isPositionTriggered;

    /// \brief flag  that enables reference angle range events
    private: bool referenceAngleRange;

    /// \brief minimum joint position.
    private: double minReferenceAngle;

    /// \brief maximum joint position
    private: double maxReferenceAngle;

    /// \brief true if the joint is currently inside the trigger condition
    private: bool isReferenceAngleTriggered;


/*
    /// \brief flag that enables velocity range events
    private: bool velocityRange;

    /// \brief minimum joint velocity trigger
    private: double minVelocity;

    /// \brief max joint velocity trigger
    private: double maxVelocity;

    /// \brief true when velocity is in the range
    private: bool isVelocityTriggered;
*/
  };
}
#endif
