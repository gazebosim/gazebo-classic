/*
 * Copyright 2011 Nate Koenig
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
/* Desc: A world state
 * Author: Nate Koenig
 */

#ifndef _WORLD_STATE_HH_
#define _WORLD_STATE_HH_

#include <string>
#include <vector>

#include "sdf/sdf.hh"
#include "physics/State.hh"
#include "physics/ModelState.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class WorldState WorldState.hh physics/WorldState.hh
    /// \brief Store state information of a physics::World object
    ///
    /// Instances of this class contain the state of a World at a specific
    /// time. World state includes the state of all models, and their
    /// children.
    class WorldState : public State
    {
      /// \brief Default constructor
      public: WorldState();

      /// \brief Constructor
      ///
      /// Generate a WorldState from an instance of a World.
      /// \param _world Pointer to a world
      public: WorldState(WorldPtr _world);

      /// \brief Destructor
      public: virtual ~WorldState();

      /// \brief Load state from SDF element
      ///
      /// Set a WorldState from an SDF element containing WorldState info.
      /// \param _elem Pointer to the WorldState SDF element.
      public: virtual void Load(sdf::ElementPtr _elem);

      /// \brief Get the number of model states
      ///
      /// Returns the number of models in this instance.
      /// \return Number of models
      public: unsigned int GetModelStateCount() const;

      /// \brief Get the state in SDF format
      ///
      /// Returns a pointer to the SDF representation of the WorldState
      /// \return Pointer to the SDF representation of the WorldState
      public: const sdf::ElementPtr &GetSDF() const;

      /// \brief Get a model state
      ///
      /// Get the state of a Model based on an index. The min index is
      /// and the max is WorldState::GetModelStateCount()
      ///
      /// \param _index Index of the model
      /// \return State of the requested Model
      public: ModelState GetModelState(unsigned int _index) const;

      /// \brief Get a model state by model name
      public: ModelState GetModelState(const std::string &_modelName) const;

      /// State of all the models.
      private: std::vector<ModelState> modelStates;

      private: sdf::ElementPtr sdf;
    };
    /// \}
  }
}
#endif
