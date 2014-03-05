/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#ifndef _GAZEBO_CONSOLE_HH_
#define _GAZEBO_CONSOLE_HH_

#include <iostream>
#include <fstream>
#include <string>

#include "gazebo/common/SingletonT.hh"
#include "gazebo/common/CommonTypes.hh"

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

    /// \brief Output a message to a log file
    #define gzlog (gazebo::common::Console::Instance()->Log())

    /// Start marker
    #define gzclr_start(clr) "\033[1;33m"

    /// End marker
    #define gzclr_end "\033[0m"

    /// \addtogroup gazebo_common Common
    /// \{

    /// \class Console Console.hh common/commom.hh
    /// \brief Message, error, warning functionality

    class Console : public SingletonT<Console>
    {
      /// \brief Default constructor
      private: Console();

      /// \brief Destructor
      private: virtual ~Console();

      /// \brief Load the message parameters
      public: void Init(const std::string &_logFilename);

      /// \brief Return true if Init has been called.
      /// \return True is initialized.
      public: bool IsInitialized() const;

      /// \brief Set quiet output
      /// \param[in] q True to prevent warning
      public: void SetQuiet(bool _q);

      /// \brief Get whether quiet output is set
      /// \return True to if quiet output is set
      public: bool GetQuiet() const;

      /// \brief Use this to output a colored message to the terminal
      /// \param[in] _lbl Text label
      /// \param[in] _color Color to make the label
      /// \return Reference to an output stream
      public: std::ostream &ColorMsg(const std::string &_lbl, int _color);

      /// \brief Use this to output a colored message to the terminal
      /// \param[in] _lbl Text label
      /// \return Reference to an output stream
      public: std::ofstream &Log();

      /// \brief Use this to output an error to the terminal
      /// \param[in] _lbl Text label
      /// \param[in] _file File containing the error
      /// \param[in] _line Line containing the error
      /// \param[in] _color Color to make the label
      /// \return Reference to an output stream
      public: std::ostream &ColorErr(const std::string &_lbl,
                  const std::string &_file, unsigned int _line, int _color);


      /// \class NullStream Animation.hh common/common.hh
      /// \brief A stream that does not output anywhere
      private: class NullStream : public std::ostream
               {
                 /// \brief constructor
                 public: NullStream() : std::ios(0), std::ostream(0) {}
               };

      /// \brief Null stream
      private: NullStream nullStream;

      /// \brief Message stream
      private: std::ostream *msgStream;

      /// \brief Error stream
      private: std::ostream *errStream;

      /// \brief Stream for a log file.
      private: std::ofstream *logStream;

      /// \brief True to silence msg output.
      private: bool quiet;

      /// \brief This is a singleton
      private: friend class SingletonT<Console>;
    };
    /// \}
  }
}
#endif
