/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#include <sdf/sdf.hh>

#include "gazebo/physics/State.hh"
#include "gazebo/physics/ModelState.hh"
#include "gazebo/util/system.hh"

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
    class GZ_PHYSICS_VISIBLE WorldState : public State
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

      /// \brief Load from a World pointer.
      ///
      /// Generate a WorldState from an instance of a World.
      /// \param[in] _world Pointer to a world
      public: void Load(const WorldPtr _world);

      /// \brief Load state from SDF element.
      ///
      /// Set a WorldState from an SDF element containing WorldState info.
      /// \param[in] _elem Pointer to the WorldState SDF element.
      public: virtual void Load(const sdf::ElementPtr _elem);

      /// \brief Set the world.
      /// \param[in] _world Pointer to the world.
      public: void SetWorld(const WorldPtr _world);

      /// \brief Get model states based on a regular expression.
      /// \param[in] _regex The regular expression.
      /// \return List of model states whose names match the regular
      /// expression.
      public: ModelState_M GetModelStates(const boost::regex &_regex) const;

      /// \brief Get the model states.
      /// \return A vector of model states.
      public: const ModelState_M &GetModelStates() const;

      /// \brief Get the number of model states.
      ///
      /// Returns the number of models in this instance.
      /// \return Number of models.
      public: unsigned int GetModelStateCount() const;

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

      /// \brief Get the vector of SDF insertions.
      /// \return A vector of SDF blocks. Each block contains the SDF of the
      /// model to be spawned in the simulation.
      public: const std::vector<std::string> &Insertions() const;

      /// \brief Set a new vector of SDF insertions.
      /// \param[in] _insertions Vector containing SDF blocks. Each block should
      /// contain the SDF of the new models spawned in the current simulation
      /// frame.
      public: void SetInsertions(const std::vector<std::string> &_insertions);

      /// \brief Get the vector of SDF deletions.
      /// \return A vector of SDF blocks. Each block contains the SDF of the
      /// model to be removed from the simulation.
      public: const std::vector<std::string> &Deletions() const;

      /// \brief Set a new vector of SDF deletions.
      /// \param[in] _deletions Vector containing SDF blocks. Each block should
      /// contain the SDF of the models removed in the current simulation frame.
      public: void SetDeletions(const std::vector<std::string> &_deletions);

      /// \brief Return true if the values in the state are zero.
      ///
      /// This will check to see if the all model states are zero.
      /// \return True if the values in the state are zero.
      public: bool IsZero() const;

      /// \brief Populate a state SDF element with data from the object.
      /// \param[out] _sdf SDF element to populate.
      public: void FillSDF(sdf::ElementPtr _sdf);

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

      /// \brief Set the simulation interations when this state was generated
      /// \param[in] _iterations Simulation iterations when the data was
      /// recorded.
      public: virtual void SetIterations(const uint64_t _iterations);

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
      public: inline friend std::ostream &operator<<(std::ostream &_out,
                  const gazebo::physics::WorldState &_state)
      {
        _out << "<state world_name='" << _state.name << "'>"
          << "<sim_time>" << _state.simTime << "</sim_time>"
          << "<wall_time>" << _state.wallTime << "</wall_time>"
          << "<real_time>" << _state.realTime << "</real_time>"
          << "<iterations>" << _state.iterations << "</iterations>";

        // List all of the inserted models
        if (_state.insertions.size() > 0)
        {
          _out << "<insertions>";
          for (std::vector<std::string>::const_iterator iter =
               _state.insertions.begin();
               iter != _state.insertions.end(); ++iter)
          {
            _out << *iter;
          }
          _out << "</insertions>";
        }

        // List all of the deleted models
        if (_state.deletions.size() > 0)
        {
          _out << "<deletions>";
          for (std::vector<std::string>::const_iterator iter =
               _state.deletions.begin();
               iter != _state.deletions.end(); ++iter)
          {
            _out << "<name>" << (*iter) << "</name>";
          }
          _out << "</deletions>";
        }

        // List the model states
        for (ModelState_M::const_iterator iter = _state.modelStates.begin();
            iter != _state.modelStates.end(); ++iter)
        {
          _out << iter->second;
        }

        _out << "</state>";

        return _out;
      }

      /// \brief State of all the models.
      private: ModelState_M modelStates;

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
