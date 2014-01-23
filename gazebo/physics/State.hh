/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
/* Desc: A generic physics state
 * Author: Nate Koenig
 */

#ifndef _STATE_HH_
#define _STATE_HH_

#include <string>

#include <sdf/sdf.hh>

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/common/Time.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class State State.hh physics/physics.hh
    /// \brief State of an entity.
    ///
    /// This is the base class for all State information.
    class State
    {
      /// \brief Default constructor
      public: State();

      /// \brief Constructor
      ///
      /// Construct a State object using some basic information.
      /// \param _name Name associated with the State information. This is
      /// typically the name of an Entity.
      /// \pararm _realTime Clock time since simulation started.
      /// \param _simTime Simulation time associated with this State info.
      public: State(const std::string &_name,
                    const common::Time &_realTime,
                    const common::Time &_simTime);

      /// \brief Destructor
      public: virtual ~State();

      /// \brief Load state from SDF element
      ///
      /// Populates the State information from data stored in an SDF::Element
      /// \param _elem Pointer to the SDF::Element
      public: virtual void Load(const sdf::ElementPtr _elem);

      /// \brief Assignment operator
      /// \param[in] _state State value
      /// \return this
      public: State &operator=(const State &_state);

      /// \brief Subtraction operator.
      /// \param[in] _pt A state to substract.
      /// \return The resulting state.
      public: State operator-(const State &_state) const;

      /// \brief Get the name associated with this State
      /// \return Name associated with this state information. Typically
      /// a name of an Entity.
      public: std::string GetName() const;

      /// \brief Set the name associated with this State.
      /// \param[in] _name Name associated with this state information.
      /// Typically the name of an Entity.
      public: void SetName(const std::string &_name);

      /// \brief Get the wall time when this state was generated
      /// \return The absolute clock time when the State data was recorded.
      public: common::Time GetWallTime() const;

      /// \brief Get the real time when this state was generated
      /// \return Clock time since simulation was stated.
      public: common::Time GetRealTime() const;

      /// \brief Get the sim time when this state was generated
      /// \return Simulation time when the data was recorded.
      public: common::Time GetSimTime() const;

      /// \brief Set the wall time when this state was generated
      /// \param[in] _time The absolute clock time when the State
      /// data was recorded.
      public: virtual void SetWallTime(const common::Time &_time);

      /// \brief Set the real time when this state was generated
      /// \param[in] _time Clock time since simulation was stated.
      public: virtual void SetRealTime(const common::Time &_time);

      /// \brief Set the sim time when this state was generated
      /// \param[in] _time Simulation time when the data was recorded.
      public: virtual void SetSimTime(const common::Time &_time);

      /// Name associated with this State
      protected: std::string name;

      /// Times for the state data
      protected: common::Time wallTime, realTime, simTime;
    };
    /// \}
  }
}
#endif
