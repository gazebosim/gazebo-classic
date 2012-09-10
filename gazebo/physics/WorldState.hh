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
/* Desc: A world state
 * Author: Nate Koenig
 */

#ifndef WORLD_STATE_HH
#define WORLD_STATE_HH

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

    /// \brief State of the World
    class WorldState : public State
    {
      /// \brief Default constructor
      public: WorldState();

      /// \brief Constructor
      public: WorldState(WorldPtr _world);

      /// \brief Destructor
      public: virtual ~WorldState();

      /// \brief Load state from SDF element
      public: virtual void Load(sdf::ElementPtr _elem);

      /// \brief Get the number of model states
      public: unsigned int GetModelStateCount() const;

      /// \brief Get the state in SDF format
      public: const sdf::ElementPtr &GetSDF() const;

      /// \brief Get a model state
      public: ModelState GetModelState(unsigned int _index) const;

      /// \brief Get a model state by model name
      public: ModelState GetModelState(const std::string &_modelName) const;

      private: std::vector<ModelState> modelStates;
      private: sdf::ElementPtr sdf;
    };
    /// \}
  }
}
#endif
