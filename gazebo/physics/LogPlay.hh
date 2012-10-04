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
/* Desc: A class to log data
 * Author: Nate Koenig
 * Date: 1 Jun 2010
 */

#ifndef _LOGPLAY_HH_
#define _LOGPLAY_HH_

#include <list>
#include <string>

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class Logplay Logplay.hh physics/Logplay.hh
    /// \brief Open and playback log files that were recorded using Logger.
    ///
    /// Use Logplay to open a log file (Logplay::Open), and access the
    /// recorded state information. Iterators are available to step through
    /// the state information. It is also possible to replay the data in a
    /// World using the Play functions. Replay involves reading and applying
    /// state information to a World.
    ///
    /// \sa Logger, State
    class Logplay
    {
      /// \brief Open a log file for reading
      ///
      /// Open a log file that was previously recorded.
      /// \param _logFile The file to load
      /// \return True if the log file was successfully loaded
      public: bool Open(const std::string &_logFile);

      /// \brief Get an iterator to the beginning of the log file.
      ///
      /// Requires a log file to be opened first.
      /// Use the iterator to step through a log file.
      /// \sa Logplay::Open.
      /// \return Iterator the beginning of the opened log file
      public: Iterator Begin();

      /// \brief Get an iterator to the end of the log file.
      ///
      /// Requires a log file to be opened first
      /// Use the iterator to step through a log file.
      /// \sa Logplay::Open.
      /// \return Iterator the end of the opened log file
      public: Iterator End();

      /// \brief Play a log file in a given World
      ///
      /// Replay a complete log file in a World.
      /// Requires a log file to be opened first, \sa Logplay::Open.
      /// \param _world Pointer to the World
      /// \return True if replay was successful
      public: bool Play(WorldPtr _world);

      /// \brief Play a log file in a given World using only data for a
      /// given entity.
      ///
      /// Replay a log file in a World, but only apply data for the given
      /// entity.
      /// Requires a log file to be opened first
      /// \sa Logplay::Open.
      /// \param _world Pointer to the World
      /// \param _entityName Name of the entity to search the log for
      /// \return True if replay was successful
      public: bool Play(WorldPtr _world, const std::string &_entityName);

      /// \brief Play a log file in a given World using only data for a
      /// list of entities.
      ///
      /// Replay a log file in a World, but only apply data for the given
      /// list of entities.
      /// Requires a log file to be opened first, \sa Logplay::Open.
      /// \param _world Pointer to the World
      /// \param _entityNames Names of the entities to search the log for
      /// \return True if replay was successful
      public: bool Play(WorldPtr _world,
                        const std::list<std::string> &_entityNames);

      /// \brief Play a segment of a log file in a given World
      ///
      /// Replay a segment of log file in a World. The segment is defined
      /// by a start and end time, where time is simulation time.
      /// Requires a log file to be opened first, \sa Logplay::Open.
      /// \param _world Pointer to the World.
      /// \param _start Start time, in simulation time.
      /// \param _stop Stop time, in simulation time.
      /// \return True if replay was successful.
      public: bool Play(WorldPtr _world, common::Time _start,
                        common::Time _stop);

      /// \brief Play a segment of a log file in a given World using only
      /// data for a given entity.
      ///
      /// Replay a segment of log file in a World, but only apply data for
      /// the given entity. The segment is defined
      /// by a start and end time, where time is simulation time.
      /// Requires a log file to be opened first, \sa Logplay::Open.
      /// \param _world Pointer to the World.
      /// \param _entityName Name of the entity to search the log for
      /// \param _start Start time, in simulation time.
      /// \param _stop Stop time, in simulation time.
      /// \return True if replay was successful.
      public: bool Play(WorldPtr _world, const std::string &_entityName,
                        common::Time _start, common::Time _stop);

      /// \brief Play a segment of a log file in a given World using
      /// only data for a list of entities.
      ///
      /// Replay a segment of a log file in a World, but only apply data for
      /// the given list of entities.
      /// Requires a log file to be opened first, \sa Logplay::Open.
      /// \param _world Pointer to the World
      /// \param _entityNames Names of the entities to search the log for
      /// \param _start Start time, in simulation time.
      /// \param _stop Stop time, in simulation time.
      /// \return True if replay was successful
      public: bool Play(WorldPtr _world,
                        const std::list<std::string> &_entityNames,
                        common::Time _start, common::Time _stop);
    };
    /// \}
  }
}

#endif
