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
/*
 * Desc: Gazebo Message
 * Author: Nathan Koenig
 * Date: 09 June 2007
 */

#ifndef GAZEBO_CONSOLE_HH
#define GAZEBO_CONSOLE_HH

#include <iostream>
#include <fstream>
#include <string>

#include "common/CommonTypes.hh"

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common Common
    /// \{
    /// \brief Output a message
    #define gzmsg (gazebo::common::Console::Instance()->ColorMsg("Msg", 32))

    /// \brief Output a debug message
    #define gzdbg (gazebo::common::Console::Instance()->ColorMsg("Dbg", 36))

    /// \brief Output a warning message
    #define gzwarn (gazebo::common::Console::Instance()->ColorErr("Warning", \
          __FILE__, __LINE__, 33))

    /// \brief Output an error message
    #define gzerr (gazebo::common::Console::Instance()->ColorErr("Error", \
          __FILE__, __LINE__, 31))

    /// Log a message
    #define gzlog (gazebo::common::Console::Instance()->Log() << "[" <<\
        __FILE__ << ":" << __LINE__ << "] ")

    /// \brief Message, error, warning, and logging functionality
    class Console
    {
      /// \brief Default constructor
      private: Console();

      /// \brief Destructor
      private: virtual ~Console();

      /// \brief Return an instance to this class
      public: static Console *Instance();

      /// \brief Load the message parameters
      public: void Load();

      /// \brief Set quiet output
      /// \param q True to prevent warning
      public: void SetQuiet(bool _q);

      /// \brief Use this to output a message to the terminal
      /// \param level Level of the message
      public: std::ostream &ColorMsg(const std::string &lbl, int color);

      /// \brief Use this to output an error to the terminal
      /// \param level Level of the message
      public: std::ostream &ColorErr(const std::string &lbl,
                  const std::string &file, unsigned int line, int color);

      /// \brief Use this to output a message to a log file
      public: std::ofstream &Log();

      /// \brief True if logging data
      private: bool logData;

      private: class NullStream : public std::ostream
               {
                 public: NullStream() : std::ios(0), std::ostream(0) {}
               };

      private: NullStream nullStream;
      private: std::ostream *msgStream;
      private: std::ostream *errStream;
      private: std::ofstream logStream;

      /// Pointer to myself
      private: static Console *myself;
    };

    /// \}
  }
}
#endif

