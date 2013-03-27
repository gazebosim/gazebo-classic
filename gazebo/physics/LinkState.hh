/*
 * Copyright 2012 Open Source Robotics Foundation
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
/* Desc: A link state
 * Author: Nate Koenig
 */

#ifndef _LINKSTATE_HH_
#define _LINKSTATE_HH_

#include <vector>
#include <string>

#include <sdf/sdf.hh>
#include "gazebo/physics/State.hh"
#include "gazebo/physics/CollisionState.hh"
#include "gazebo/math/Pose.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class LinkState LinkState.hh physics/physics.hh
    /// \brief Store state information of a physics::Link object
    ///
    /// This class captures the entire state of a Link at one
    /// specific time during a simulation run.
    ///
    /// State of a Link includes the state of itself all its child Collision
    /// entities.
    class LinkState : public State
    {
      /// \brief Default constructor
      public: LinkState();

      /// \brief Constructor
      ///
      /// Build a LinkState from an existing Link.
      /// \param[in] _model Pointer to the Link from which to gather state
      /// info.
      public: explicit LinkState(const LinkPtr _link);

      /// \brief Constructor
      ///
      /// Build a LinkState from SDF data
      /// \param[in] _sdf SDF data to load a link state from.
      public: explicit LinkState(const sdf::ElementPtr _sdf);

      /// \brief Destructor.
      public: virtual ~LinkState();

      /// \brief Load state from SDF element.
      ///
      /// Load LinkState information from stored data in and SDF::Element.
      /// \param[in] _elem Pointer to the SDF::Element containing state info.
      public: virtual void Load(const sdf::ElementPtr _elem);

      /// \brief Get the link pose.
      /// \return The math::Pose of the Link.
      public: const math::Pose &GetPose() const;

      /// \brief Get the link velocity.
      /// \return The velocity represented as a math::Pose.
      public: const math::Pose &GetVelocity() const;

      /// \brief Get the link acceleration.
      /// \return The acceleration represented as a math::Pose.
      public: const math::Pose &GetAcceleration() const;

      /// \brief Get the force applied to the Link.
      /// \return Magnitude of the force.
      public: const math::Pose &GetWrench() const;

      /// \brief Get the number of link states.
      ///
      /// This returns the number of Collisions recorded.
      /// \return Number of CollisionState recorded.
      public: unsigned int GetCollisionStateCount() const;

      /// \brief Get a collision state.
      ///
      /// Get a Collision State based on an index, where index is in the
      /// range of  0...LinkState::GetCollisionStateCount.
      /// \param[in] _index Index of the CollisionState.
      /// \return State of the Collision.
      /// \throws common::Exception When _index is invalid.
      public: CollisionState GetCollisionState(unsigned int _index) const;

      /// \brief Get a link state by link name.
      ///
      /// Searches through all CollisionStates.
      /// Returns the CollisionState with the matching name, if any.
      /// \param[in] _collisionName Name of the CollisionState
      /// \return State of the Collision.
      /// \throws common::Exception When _collisionName is invalid
      public: CollisionState GetCollisionState(
                  const std::string &_collisionName) const;

      /// \brief Get the collision states.
      /// \return A vector of collision states.
      public: const std::vector<CollisionState> &GetCollisionStates() const;

      /// \brief Return true if the values in the state are zero.
      /// \return True if the values in the state are zero.
      public: bool IsZero() const;

      /// \brief Populate a state SDF element with data from the object.
      /// \param[out] _sdf SDF element to populate.
      public: void FillSDF(sdf::ElementPtr _sdf);

      /// \brief Assignment operator
      /// \param[in] _state State value
      /// \return this
      public: LinkState &operator=(const LinkState &_state);

      /// \brief Subtraction operator.
      /// \param[in] _pt A state to substract.
      /// \return The resulting state.
      public: LinkState operator-(const LinkState &_state) const;

      /// \brief Addition operator.
      /// \param[in] _pt A state to add.
      /// \return The resulting state.
      public: LinkState operator+(const LinkState &_state) const;

      /// \brief Stream insertion operator
      /// \param[in] _out output stream
      /// \param[in] _state Link state to output
      /// \return the stream
      public: friend std::ostream &operator<<(std::ostream &_out,
                                     const gazebo::physics::LinkState &_state)
      {
        _out << "    <link name='" << _state.name << "'>\n";
        _out << "      <pose>" << _state.pose << "</pose>\n";
        _out << "      <velocity>" << _state.velocity << "</velocity>\n";
        _out << "      <acceleration>" << _state.acceleration
             << "</acceleration>\n";
        _out << "      <wrench>" << _state.wrench << "</wrench>\n";

        for (std::vector<CollisionState>::const_iterator iter =
             _state.collisionStates.begin();
             iter != _state.collisionStates.end(); ++iter)
        {
          _out << *iter;
        }

        _out << "    </link>\n";

        return _out;
      }

      /// \brief 3D pose of the link relative to the model.
      private: math::Pose pose;

      /// \brief Velocity of the link (linear and angular).
      private: math::Pose velocity;

      /// \brief Acceleration of the link (linear and angular).
      private: math::Pose acceleration;

      /// \brief Force on the link(linear and angular).
      private: math::Pose wrench;

      /// \brief State of all the child Collision objects.
      private: std::vector<CollisionState> collisionStates;
    };
    /// \}
  }
}
#endif
