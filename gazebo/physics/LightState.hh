/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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

#ifndef _GAZEBO_LIGHT_STATE_HH_
#define _GAZEBO_LIGHT_STATE_HH_

#include <iomanip>

#include "gazebo/physics/State.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class LightState LightState.hh physics/physics.hh
    /// \brief Store state information of a Light object
    ///
    /// This class captures the entire state of a Light at one
    /// specific time during a simulation run.
    class GZ_PHYSICS_VISIBLE LightState : public State
    {
      /// \brief Default constructor.
      public: LightState() = default;

      /// \brief Constructor.
      ///
      /// Build a LightState from an existing Light.
      /// \param[in] _light Pointer to the light from which to gather state
      /// info.
      /// \param[in] _realTime Real time stamp.
      /// \param[in] _simTime Sim time stamp.
      /// \param[in] _iterations Simulation iterations.
      public: LightState(const LightPtr _light, const common::Time &_realTime,
                  const common::Time &_simTime, const uint64_t _iterations);

      /// \brief Constructor
      ///
      /// Build a LightState from SDF data
      /// \param[in] _sdf SDF data to load a light state from.
      public: explicit LightState(const sdf::ElementPtr _sdf);

      /// \brief Load state from SDF element.
      ///
      /// Load LightState information from stored data in and SDF::Element
      /// \param[in] _elem Pointer to the SDF::Element containing state info.
      public: virtual void Load(const sdf::ElementPtr _elem);

      /// \brief Load state from Light pointer.
      ///
      /// Build a LightState from an existing Light.
      /// \param[in] _light Pointer to the light from which to gather state
      /// info.
      /// \param[in] _realTime Real time stamp.
      /// \param[in] _simTime Sim time stamp.
      /// \param[in] _iterations Simulation iterations.
      public: void Load(const LightPtr _light, const common::Time &_realTime,
                  const common::Time &_simTime, const uint64_t _iterations);

      /// \brief Get the stored light pose.
      /// \return Pose of the Light.
      public: const ignition::math::Pose3d Pose() const;

      /// \brief Return true if the values in the state are zero.
      /// \return True if the values in the state are zero.
      public: bool IsZero() const;

      /// \brief Populate a state SDF element with data from the object.
      /// \param[out] _sdf SDF element to populate.
      public: void FillSDF(sdf::ElementPtr _sdf);

      /// \brief Assignment operator
      /// \param[in] _state State value
      /// \return this
      public: LightState &operator=(const LightState &_state);

      /// \brief Subtraction operator.
      /// \param[in] _pt A state to substract.
      /// \return The resulting state.
      public: LightState operator-(const LightState &_state) const;

      /// \brief Addition operator.
      /// \param[in] _pt A state to substract.
      /// \return The resulting state.
      public: LightState operator+(const LightState &_state) const;

      /// \brief Stream insertion operator.
      /// \param[in] _out output stream.
      /// \param[in] _state Light state to output.
      /// \return The stream.
      public: inline friend std::ostream &operator<<(std::ostream &_out,
                  const gazebo::physics::LightState &_state)
      {
        ignition::math::Vector3d q(_state.pose.Rot().Euler());
        _out << std::fixed <<std::setprecision(3)
          << "<light name='" << _state.GetName() << "'>"
          << "<pose>"
          << _state.pose.Pos().X() << " "
          << _state.pose.Pos().Y() << " "
          << _state.pose.Pos().Z() << " "
          << q.X() << " "
          << q.Y() << " "
          << q.Z() << " "
          << "</pose>";

        _out << "</light>";

        return _out;
      }

      /// \brief Pose of the light.
      private: ignition::math::Pose3d pose;
    };

    /// \}
  }
}
#endif

