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

#ifndef _GAZEBO_CONSOLE_HH_
#define _GAZEBO_CONSOLE_HH_

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <boost/thread.hpp>
#endif
#include "gazebo/common/SingletonT.hh"
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common Common
    /// \{
    /// \brief Output a message
    #define gzmsg (gazebo::common::Console::msg())

    /// \brief Output a debug message
    #define gzdbg (gazebo::common::Console::dbg(__FILE__, __LINE__))

    /// \brief Output a warning message
    #define gzwarn (gazebo::common::Console::warn(__FILE__, __LINE__))

    /// \brief Output an error message
    #define gzerr (gazebo::common::Console::err(__FILE__, __LINE__))

    /// \brief Output a message to a log file
    #define gzlog (gazebo::common::Console::log())

    /// \brief Initialize log file with filename given by _str.
    /// If called twice, it will close currently in use and open a new
    /// log file.
    /// \param[in] _prefix Prefix added to the directory where the log file
    /// will be created.
    /// \param[in] _str Name of log file for gzlog messages.
    #define gzLogInit(_prefix, _str) \
        (gazebo::common::Console::log.Init(_prefix, _str))

    /// \brief Get the full path of the directory where the log files are stored
    /// \return Full path of the directory
    #define gzLogDirectory() (gazebo::common::Console::log.GetLogDirectory())

    /// \class FileLogger FileLogger.hh common/common.hh
    /// \brief A logger that outputs messages to a file.
    class GAZEBO_VISIBLE FileLogger : public std::ostream
    {
      /// \brief Constructor.
      /// \param[in] _filename Filename to write into. If empty,
      /// FileLogger::Init must be called separately.
      public: FileLogger(const std::string &_filename = "");

      /// \brief Destructor.
      public: virtual ~FileLogger();

      /// \brief Initialize the file logger.
      /// \param[in] _prefix Prefix added to the directory where the log file
      /// will be created.
      /// \param[in] _filename Name and path of the log file to write output
      /// into.
      public: void Init(const std::string &_prefix,
                        const std::string &_filename);

      /// \brief Output a filename and line number, then return a reference
      /// to the logger.
      /// \return Reference to this logger.
      public: virtual FileLogger &operator()();

      /// \brief Output a filename and line number, then return a reference
      /// to the logger.
      /// \param[in] _file Filename to output.
      /// \param[in] _line Line number in the _file.
      /// \return Reference to this logger.
      public: virtual FileLogger &operator()(
                  const std::string &_file, int _line);

      /// \brief Get the full path of the directory where all the log files
      /// are stored.
      /// \return Full path of the directory.
      public: std::string GetLogDirectory() const;

      /// \brief Get the port of the master.
      /// \return The port of the master.
      private: static std::string GetMasterPort();

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
                   /// \return Return 0 on success.
                   public: virtual int sync();

                   /// \brief Stream to output information into.
                   public: std::ofstream *stream;
                 };

      /// \brief Stores the full path of the directory where all the log files
      /// are stored.
      private: std::string logDirectory;
    };

    /// \class Logger Logger.hh common/common.hh
    /// \brief Terminal logger.
    class GAZEBO_VISIBLE Logger : public std::ostream
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

      /// \brief Access operator.
      /// \return Reference to this logger.
      public: virtual Logger &operator()();

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
                   /// \param[in] _type Output destination type
                   /// (STDOUT, or STDERR)
                   /// \param[in] _color Color of the output stream.
                   public: Buffer(LogType _type, int _color);

                   /// \brief Destructor.
                   public: virtual ~Buffer();

                   /// \brief Sync the stream (output the string buffer
                   /// contents).
                   /// \return Return 0 on success.
                   public: virtual int sync();

                   /// \brief Destination type for the messages.
                   public: LogType type;

                   /// \brief ANSI color code using Select Graphic Rendition
                   /// parameters (SGR). See
                   /// http://en.wikipedia.org/wiki/ANSI_escape_code#Colors
                   public: int color;
                 };

      /// \brief Color for the output.
      public: int color;

      /// \brief Prefix to use when logging to file.
      private: std::string prefix;
    };

    /// \class Console Console.hh common/common.hh
    /// \brief Container for loggers, and global logging options
    /// (such as verbose vs. quiet output).
    class GAZEBO_VISIBLE Console
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
