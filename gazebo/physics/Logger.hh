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

#ifndef _LOGGER_HH_
#define _LOGGER_HH_

#include <vector>
#include <fstream>
#include <string>

#include "common/SingletonT.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class Logger Logger.hh physics/Logger.hh
    /// \brief Handles logging of data to disk
    ///
    /// The Logger class is a Singleton that manages data logging of any
    /// entity within a running simulation. An entity may be a World, Model,
    /// or any of their child entities. This class only writes log files,
    /// see LogPlay for playback functionality.
    ///
    /// State information for an entity may be logged through the Logger::Add
    /// function, and stopped through the Logger::Remove function. Data may
    /// be logged into a single file, or split into many separate files by
    /// specifying different filenames for the Logger::Add function.
    ///
    /// The Logger is updated at the start of each simulation step. This
    /// guarantees that all data is stored.
    ///
    /// \sa Logplay, State
    class Logger : public SingletonT<Logger>
    {
      /// \brief Constructor
      private: Logger();

      /// \brief Destructor
      private: virtual ~Logger();

      /// \brief Add an entity to a log file.
      ///
      /// Add a new entity to a log. An entity can be any valid named object
      /// in the world, including the world itself. Duplicate additions are
      /// ignored. Entites can be added to the same file by passing
      /// specifying the same _filename.
      /// \param _entityName Name of the entity to log.
      /// \param _filename Filename of the log file.
      /// \return True if the Entity was added to a log. False if the entity
      /// is already being logged.
      public: bool Add(const std::string &_entityName,
                       const std::string &_filename);

      /// \brief Remove an entity from a log
      ///
      /// Removes an entity from the logger. The stops data recording for
      /// the entity and all its children. For example, specifying a world
      /// will stop all data logging.
      /// \param _entity Name of the entity to stop logging
      /// \return True if the entity existed and was removed. False if the
      /// entity was not registered with the logger.
      public: bool Remove(const std::string &_entity);

      /// \brief Update the log files
      ///
      /// Captures the current state of all registered entities, and outputs
      /// the data to their respective log files.
      private: void Update();

      /// \brief A logger for an entitiy
      private: class LogObj
               {
                 public: LogObj(const std::string &entityName,
                                const std::string &filename);
                 public: virtual ~LogObj();
                 public: void Update();
                 public: std::string GetEntityName() const;

                 public: bool valid;
                 private: Entity *entity;
                 private: std::fstream logFile;
                 private: Time startSimTime;
                 private: Time startRealTime;
               };

      private: std::vector<LogObj*> logObjects;

      private: friend class SingletonT<Logger>;
    };
    /// \}
  }
}
#endif
