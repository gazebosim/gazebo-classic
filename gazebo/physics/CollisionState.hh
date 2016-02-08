/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
/* Desc: A collision state
 * Author: Nate Koenig
 */

#ifndef _COLLISIONSTATE_HH_
#define _COLLISIONSTATE_HH_

#include "gazebo/physics/State.hh"
#include "gazebo/math/Pose.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class CollisionState CollisionState.hh physics/phyiscs.hh
    /// \brief Store state information of a physics::Collision object
    ///
    /// This class captures the entire state of a Collision at one
    /// specific time during a simulation run.
    ///
    /// State of a Collision is its Pose.
    class GZ_PHYSICS_VISIBLE CollisionState : public State
    {
      /// \brief Default constructor
      public: CollisionState();

      /// \brief Constructor
      ///
      /// Build a CollisionState from an existing Collision.
      /// \param[in] _model Pointer to the Link from which to gather state
      /// info.
      public: explicit CollisionState(const CollisionPtr _collision);

      /// \brief Constructor
      ///
      /// Build a CollisionState from SDF data
      /// \param[in] _sdf SDF data to load a collision state from.
      public: explicit CollisionState(const sdf::ElementPtr _sdf);

      /// \brief Destructor
      public: virtual ~CollisionState();

      /// \brief Load state from SDF element
      ///
      /// Load CollisionState information from stored data in and SDF::Element
      /// \param[in] _elem Pointer to the SDF::Element containing state info.
      public: virtual void Load(const sdf::ElementPtr _elem);

      /// \brief Get the Collision pose
      /// \return The pose of the CollisionState
      public: const math::Pose &GetPose() const;

      /// \brief Return true if the values in the state are zero.
      /// \return True if the values in the state are zero.
      public: bool IsZero() const;

      /// \brief Populate a state SDF element with data from the object.
      /// \param[out] _sdf SDF element to populate.
      public: void FillSDF(sdf::ElementPtr _sdf);

      /// \brief Assignment operator
      /// \param[in] _state State value
      /// \return Reference to this
      public: CollisionState &operator=(const CollisionState &_state);

      /// \brief Subtraction operator.
      /// \param[in] _pt A state to substract.
      /// \return The resulting state.
      public: CollisionState operator-(const CollisionState &_state) const;

      /// \brief Addition operator.
      /// \param[in] _pt A state to add.
      /// \return The resulting state.
      public: CollisionState operator+(const CollisionState &_state) const;

      /// \brief Stream insertion operator
      /// \param[in] _out output stream
      /// \param[in] _state Collision state to output
      /// \return the stream
      public: inline friend std::ostream &operator<<(std::ostream &_out,
                  const gazebo::physics::CollisionState &_state)
      {
        _out << "<collision name='" << _state.name << "'>"
             << "<pose>" << _state.pose << "</pose>";
        _out << "</collision>";

        return _out;
      }

      /// \brief Pose of the Collision object.
      private: math::Pose pose;
    };
    /// \}
  }
}
#endif
