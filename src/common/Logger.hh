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
/* Desc: A class to log data
 * Author: Nate Koenig
 * Date: 1 Jun 2010
 */

#ifndef LOGGER_HH
#define LOGGER_HH

#include <vector>
#include <fstream>
#include <string>

#include "SingletonT.hh"

namespace gazebo
{
  class Logger : public SingletonT<Logger>
  {
    /// \brief Constructor
    public: Logger();

    /// \brief Destructor
    public: virtual ~Logger();

    /// \brief Add a log file
    public: void AddLog(const std::string &model, const std::string &filename);

    /// \brief Remove a log
    public: void RemoveLog(const std::string &entity);

    /// \brief Update the log files
    public: void Update();

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

    private: friend class DestroyerT<Logger>;
    private: friend class SingletonT<Logger>;
  };
}
#endif
