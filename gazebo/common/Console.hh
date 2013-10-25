/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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
#include <sstream>
#include <string>

#include <boost/thread.hpp>
#include "gazebo/common/SingletonT.hh"
#include "gazebo/common/CommonTypes.hh"

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common Common
    /// \{
    /// \brief Output a message
    #define gzmsg (gazebo::common::Console::msg)

    /// \brief Output a debug message
    #define gzdbg (gazebo::common::Console::dbg(__FILE__, __LINE__))

    /// \brief Output a warning message
    #define gzwarn (gazebo::common::Console::warn)

    /// \brief Output an error message
    #define gzerr (gazebo::common::Console::err(__FILE__, __LINE__))

    /// \brief Output a message to a log file
    #define gzlog (gazebo::common::Console::log)

    #define gzLogInit(_str) (gazebo::common::Console::log.Init(_str))

    /// \brief A logger that outputs messages to a file.
    class FileLogger : public std::ostream
    {
      /// \brief Constructor.
      /// \param[in] _filename Filename to write into. If empty,
      /// FileLogger::Init must be called separately.
      public: FileLogger(const std::string &_filename = "");

      /// \brief Destructor.
      public: virtual ~FileLogger();

      /// \brief Initialize the file logger.
      /// \param[in] _filename Name and path of the log file to write output
      /// into.
      public: void Init(const std::string &_filename);

      /// \brief String buffer for the file logger.
      protected: class Buffer : public std::stringbuf
                 {
                   /// \brief Constructor.
                   /// \param[in] _filename Filename to write into.
                   public: Buffer(const std::string &_filename);

                   /// \brief Destructor.
                   public: virtual ~Buffer();

                   /// \brief Sync the stream (output the string buffer
                   /// contents).
                   public: virtual int sync();

                   /// \brief Stream to output information into.
                   public: std::ofstream *stream;
                 };
    };

    /// \brief Terminal logger.
    class Logger : public std::ostream
    {
      /// \enum LogType.
      /// \brief Output destination type.
      public: enum LogType
              {
                /// \brief Output to stdout.
                STDOUT,
                /// \brief Output to stderr.
                STDERR
              };

      /// \brief Constructor.
      /// \param[in] _prefix String to use as prefix when logging to file.
      /// \param[in] _color Color of the output stream.
      /// \param[in] _type Output destination type (STDOUT, or STDERR)
      public: Logger(const std::string &_prefix, int _color, LogType _type);

      /// \brief Destructor.
      public: virtual ~Logger();

      /// \brief Output a filename and line number, then return a reference
      /// to the logger.
      /// \param[in] _file Filename to output.
      /// \param[in] _line Line number in the _file.
      /// \return Reference to this logger.
      public: virtual Logger &operator()(
                  const std::string &_file, int _line);

      /// \brief String buffer for the base logger.
      protected: class Buffer : public std::stringbuf
                 {
                   /// \brief Constructor.
                   /// \param[in] _prefix String to use as prefix when
                   /// logging to file.
                   /// \param[in] _color Color of the output stream.
                   /// \param[in] _type Output destination type
                   /// (STDOUT, or STDERR)
                   public: Buffer(const std::string &_prefix,
                               int _color, LogType _type);

                   /// \brief Destructor.
                   public: virtual ~Buffer();

                   /// \brief Sync the stream (output the string buffer
                   /// contents).
                   public: virtual int sync();

                   /// \brief Color for the output.
                   public: int color;

                   /// \brief Destination type for the messages.
                   public: LogType type;

                   /// \brief Prefix to use when logging to file.
                   private: std::string prefix;
                 };
    };

    /// \class Console Console.hh common/common.hh
    /// \brief Container for loggers, and global logging options
    /// (such as verbose vs. quiet output).
    class Console
    {
      /// \brief Set quiet output.
      /// \param[in] q True to prevent warning.
      public: static void SetQuiet(bool _q);

      /// \brief Get whether quiet output is set.
      /// \return True to if quiet output is set.
      public: static bool GetQuiet();

      /// \brief Global instance of the message logger.
      public: static Logger msg;

      /// \brief Global instance of the error logger.
      public: static Logger err;

      /// \brief Global instance of the debug logger.
      public: static Logger dbg;

      /// \brief Global instance of the warning logger.
      public: static Logger warn;

      /// \brief Global instance of the file logger.
      public: static FileLogger log;

      /// \brief Indicates if console messages should be quiet.
      private: static bool quiet;
    };
    /// \}
  }
}
#endif
