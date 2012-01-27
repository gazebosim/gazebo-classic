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
/* Desc: The world; all models are collected here
 * Author: Andrew Howard and Nate Koenig
 * Date: 3 Apr 2007
 */

#ifndef WORLD_STATE_HH
#define WORLD_STATE_HH

namespace gazebo
{
  namespace physics
  {
    class WorldState
    {
      /// \brief Constructor
      public: WorldState();

      /// \brief Destructor
      public: virtual ~WorldState();

      /// \brief Get the wall time when this state was generated
      public: common::Time GetWallTime() const;

      /// \brief Get the real time when this state was generated
      public: common::Time GetRealTime() const;

      /// \brief Get the sim time when this state was generated
      public: common::Time GetSimTime() const;

      /// \brief Set the wall time when this state was generated
      public: void SetWallTime(const common::Time &_time) const;

      /// \brief Set the real time when this state was generated
      public: void SetRealTime(const common::Time &_time) const;

      /// \brief Set the sim time when this state was generated
      public: void SetSimTime(const common::Time &_time) const;

      /// \brief Get the number of model states
      public: unsigned int GetModelStateCount() const;

      /// \brief Get a model state
      public: const ModelState &GetModelStateCount(unsigned int _index) const;

      /// \brief Get a model state by model name
      public: const ModelState &GetModelStateCount(
                  const std::string &_modelName) const;

      private: common::Time wallTime, realTime, simTime;
      private: std::vector<ModelState> modelStates;
    };
  }
}
#endif
