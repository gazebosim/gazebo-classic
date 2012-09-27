/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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

#ifndef __LINK_STATE_HH__
#define __LINK_STATE_HH__

#include <vector>
#include <string>

#include "sdf/sdf.hh"
#include "physics/State.hh"
#include "physics/CollisionState.hh"
#include "math/Pose.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class LinkState LinkState.hh physics/LinkState.hh
    /// \brief Keeps track of states of a physics::Link
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
      /// \param _model Pointer to the Link from which to gather state
      /// info.
      public: LinkState(const LinkPtr _link);

      /// \brief Destructor
      public: virtual ~LinkState();

      /// \brief Load state from SDF element
      ///
      /// Load LinkState information from stored data in and SDF::Element
      /// \param _elem Pointer to the SDF::Element containing state info.
      public: virtual void Load(sdf::ElementPtr _elem);

      /// \brief Get the link pose
      /// \return The math::Pose of the Link
      public: math::Pose GetPose() const;

      /// \brief Get the number of link states
      ///
      /// This returns the number of Collisions recorded.
      /// \return Number of CollisionState recorded
      public: unsigned int GetCollisionStateCount() const;

      /// \brief Get a collision state
      ///
      /// Get a Collision State based on an index, where index is in the
      /// range of  0...LinkState::GetCollisionStateCount
      /// \param _index Index of the CollisionState
      /// \return State of the Collision
      public: CollisionState GetCollisionState(unsigned int _index) const;

      /// \brief Get a link state by link name
      ///
      /// Searches through all CollisionStates.
      /// Returns the CollisionState with the matching name, if any.
      /// \param _collisionName Name of the CollisionState
      /// \return State of the Collision.
      public: CollisionState GetCollisionState(
                  const std::string &_collisionName) const;

      /// \brief Fill a State SDF element with state info
      ///
      /// Stored state information into an SDF::Element pointer.
      /// \param _elem Pointer to the SDF::Element which recieves the data.
      public: void FillStateSDF(sdf::ElementPtr _elem);

      /// \brief Update a Link SDF element with this state info
      ///
      /// Set the values in a Links's SDF::Element with the information
      /// stored in this instance.
      /// \param _elem Pointer to a Links's SDF::Element
      public: void UpdateLinkSDF(sdf::ElementPtr _elem);

      /// 3D pose of the link relative to the model
      private: math::Pose pose;

      /// Velocity of the link (linear and angular)
      private: math::Pose velocity;

      /// Acceleration of the link (linear and angular)
      private: math::Pose acceleration;

      // Forces on the link(linear and angular)
      private: std::vector<math::Pose> forces;

      private: std::vector<CollisionState> collisionStates;
    };
    /// \}
  }
}
#endif
