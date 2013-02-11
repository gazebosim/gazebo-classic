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
/* Desc: A world state
 * Author: Nate Koenig
 */

#ifndef _WORLDSTATE_HH_
#define _WORLDSTATE_HH_

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

    /// \class WorldState WorldState.hh physics/physics.hh
    /// \brief Store state information of a physics::World object
    ///
    /// Instances of this class contain the state of a World at a specific
    /// time. World state includes the state of all models, and their
    /// children.
    class WorldState : public State
    {
      /// \brief Default constructor
      public: WorldState();

      /// \brief Constructor.
      ///
      /// Generate a WorldState from an instance of a World.
      /// \param[in] _world Pointer to a world
      public: explicit WorldState(const WorldPtr _world);

      /// \brief Constructor
      ///
      /// Build a WorldState from SDF data
      /// \param[in] _sdf SDF data to load a world state from.
      public: explicit WorldState(const sdf::ElementPtr _sdf);

      /// \brief Destructor.
      public: virtual ~WorldState();

      /// \brief Load state from SDF element.
      ///
      /// Set a WorldState from an SDF element containing WorldState info.
      /// \param[in] _elem Pointer to the WorldState SDF element.
      public: virtual void Load(const sdf::ElementPtr _elem);

      /// \brief Set the world.
      /// \param[in] _world Pointer to the world.
      public: void SetWorld(const WorldPtr _world);

      /// \brief Get the model states.
      /// \return A vector of model states.
      public: const std::vector<ModelState> &GetModelStates() const;

      /// \brief Get the number of model states.
      ///
      /// Returns the number of models in this instance.
      /// \return Number of models.
      public: unsigned int GetModelStateCount() const;

      /// \brief Get a model state.
      ///
      /// Get the state of a Model based on an index. The min index is
      /// and the max is WorldState::GetModelStateCount().
      ///
      /// \param[in] _index Index of the model.
      /// \return State of the requested Model.
      public: ModelState GetModelState(unsigned int _index) const;

      /// \brief Get a model state by model name.
      /// \param[in] _modelName Name of the model state to get.
      /// \return The model state.
      /// \throws common::Exception When the _modelName doesn't exist.
      public: ModelState GetModelState(const std::string &_modelName) const;

      /// \brief Return true if WorldState has a ModelState with the given
      /// name.
      /// \param[in] _modelName Name of the model to search for.
      /// \return True if the ModelState exists.
      public: bool HasModelState(const std::string &_modelName) const;

      /// \brief Return true if the values in the state are zero.
      ///
      /// This will check to see if the all model states are zero.
      /// \return True if the values in the state are zero.
      public: bool IsZero() const;

      /// \brief Populate a state SDF element with data from the object.
      /// \param[out] _sdf SDF element to populate.
      public: void FillSDF(sdf::ElementPtr _sdf);

      /// \brief Assignment operator
      /// \param[in] _state State value
      /// \return Reference to this
      public: WorldState &operator=(const WorldState &_state);

      /// \brief Subtraction operator.
      /// \param[in] _pt A state to substract.
      /// \return The resulting state.
      public: WorldState operator-(const WorldState &_state) const;

      /// \brief Addition operator.
      /// \param[in] _pt A state to add.
      /// \return The resulting state.
      public: WorldState operator+(const WorldState &_state) const;

      /// \brief Stream insertion operator
      /// \param[in] _out output stream
      /// \param[in] _state World state to output
      /// \return the stream
      public: friend std::ostream &operator<<(std::ostream &_out,
                                 const gazebo::physics::WorldState &_state)
      {
        _out << "<state world_name='" << _state.name << "'>\n";
        _out << "  <sim_time>" << _state.simTime << "</sim_time>\n";
        _out << "  <wall_time>" << _state.wallTime << "</wall_time>\n";
        _out << "  <real_time>" << _state.realTime << "</real_time>\n";

        // List all of the inserted models
        if (_state.insertions.size() > 0)
        {
          _out << "  <insertions>\n";
          for (std::vector<std::string>::const_iterator iter =
               _state.insertions.begin();
               iter != _state.insertions.end(); ++iter)
          {
            _out << *iter << "\n";
          }
          _out << "  </insertions>\n";
        }

        // List all of the deleted models
        if (_state.deletions.size() > 0)
        {
          _out << "  <deletions>\n";
          for (std::vector<std::string>::const_iterator iter =
               _state.deletions.begin();
               iter != _state.deletions.end(); ++iter)
          {
            _out << "    <name>" << (*iter) << "</name>\n";
          }
          _out << "  </deletions>\n";
        }

        // List the model states
        for (std::vector<ModelState>::const_iterator iter =
            _state.modelStates.begin(); iter != _state.modelStates.end();
            ++iter)
        {
          _out << *iter;
        }

        _out << "</state>\n";

        return _out;
      }

      /// \brief State of all the models.
      private: std::vector<ModelState> modelStates;

      /// \brief List of new added models. The
      /// value is the SDF that describes the model.
      private: std::vector<std::string> insertions;

      /// \brief List of deleted models. Each string is the name of the
      /// deleted model.
      private: std::vector<std::string> deletions;

      /// \brief Pointer to the world.
      private: WorldPtr world;
    };
    /// \}
  }
}
#endif
