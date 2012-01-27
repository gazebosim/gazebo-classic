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
/* Desc: A generic physics state
 * Author: Nate Koenig
 */

#ifndef STATE_HH
#define STATE_HH

#include <string>

#include "physics/PhysicsTypes.hh"
#include "common/Time.hh"

namespace gazebo
{
  namespace physics
  {
    class State
    {
      /// \brief Default constructor
      public: State();

      /// \brief Constructor
      public: State(const std::string &_name,
                    const common::Time &_realTime,
                    const common::Time &_simTime);

      /// \brief Destructor
      public: virtual ~State();

      /// \brief Get the name of the state
      public: std::string GetName() const;

      /// \brief Get the wall time when this state was generated
      public: common::Time GetWallTime() const;

      /// \brief Get the real time when this state was generated
      public: common::Time GetRealTime() const;

      /// \brief Get the sim time when this state was generated
      public: common::Time GetSimTime() const;

      private: std::string name;
      private: common::Time wallTime, realTime, simTime;
    };
  }
}
#endif
