/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
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
