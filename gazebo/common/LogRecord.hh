/*
 * Copyright 2012 Open Source Robotics Foundation
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

#ifndef _LOGRECORD_HH_
#define _LOGRECORD_HH_

#include <fstream>
#include <string>
#include <map>
#include <boost/thread.hpp>
#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/insert_linebreaks.hpp>
#include <boost/archive/iterators/transform_width.hpp>
#include <boost/archive/iterators/ostream_iterator.hpp>
#include <boost/filesystem.hpp>

#include "common/Event.hh"
#include "common/SingletonT.hh"

#define GZ_LOG_VERSION "1.0"

namespace gazebo
{
  namespace common
  {
    /// addtogroup gazebo_common
    /// \{

    /// \class LogRecord LogRecord.hh common/common.hh
    /// \brief Handles logging of data to disk
    ///
    /// The LogRecord class is a Singleton that manages data logging of any
    /// entity within a running simulation. An entity may be a World, Model,
    /// or any of their child entities. This class only writes log files,
    /// see LogPlay for playback functionality.
    ///
    /// State information for an entity may be logged through the LogRecord::Add
    /// function, and stopped through the LogRecord::Remove function. Data may
    /// be logged into a single file, or split into many separate files by
    /// specifying different filenames for the LogRecord::Add function.
    ///
    /// The LogRecord is updated at the start of each simulation step. This
    /// guarantees that all data is stored.
    ///
    /// \sa Logplay, State
    class LogRecord : public SingletonT<LogRecord>
    {
      /// \brief Constructor
      private: LogRecord();

      /// \brief Destructor
      private: virtual ~LogRecord();

      /// \brief Initialize logging into a subdirectory.
      ///
      /// Init may only be called once, False will be returned if called
      /// multiple times.
      /// \param[in] _subdir Directory to record to
      /// \return True if successful.
      public: bool Init(const std::string &_subdir);

      /// \brief Add an object to a log file.
      ///
      /// Add a new object to a log. An object can be any valid named object
      /// in simulation, including the world itself. Duplicate additions are
      /// ignored. Objects can be added to the same file by
      /// specifying the same _filename.
      /// \param[in] _name Name of the object to log.
      /// \param[in] _filename Filename of the log file.
      /// \param[in] _logCallback Function used to log data for the object.
      /// Typically an object will have a log function that outputs data to
      /// the provided ofstream.
      /// \throws Exception
      public: void Add(const std::string &_name, const std::string &_filename,
                    boost::function<bool (std::ostringstream &)> _logCallback);

      /// \brief Remove an entity from a log
      ///
      /// Removes an entity from the logger. The stops data recording for
      /// the entity and all its children. For example, specifying a world
      /// will stop all data logging.
      /// \param[in] _name Name of the log
      /// \return True if the entity existed and was removed. False if the
      /// entity was not registered with the logger.
      public: bool Remove(const std::string &_name);

      /// \brief Stop the logger.
      public: void Stop();

      /// \brief Set whether logging should pause. A paused state means the
      /// log file is still open, but data is not written to it.
      /// \param[in] _paused True to pause data logging.
      /// \sa LogRecord::GetPaused
      public: void SetPaused(bool _paused);

      /// \brief Get whether logging is paused.
      /// \return True if logging is paused.
      /// \sa LogRecord::SetPaused
      public: bool GetPaused() const;

      /// \brief Start the logger.
      /// \param[in] _encoding The type of encoding (txt, or bz2).
      public: bool Start(const std::string &_encoding="bz2");

      /// \brief Get the encoding used.
      /// \return Either [txt, or bz2], where txt is plain txt and bz2 is
      /// bzip2 compressed data with Base64 encoding.
      public: const std::string &GetEncoding() const;

      /// \brief Update the log files
      ///
      /// Captures the current state of all registered entities, and outputs
      /// the data to their respective log files.
      private: void Update();

      /// \brief Run the Write loop.
      private: void Run();

      /// \brief Clear and delete the log buffers.
      private: void ClearLogs();

      /// \brief Write the header to file.
      // private: void WriteHeader();

      /// \cond
      private: class Log
      {
        /// \brief Constructor
        /// \param[in] _parent Pointer to the LogRecord parent.
        /// \param[in] _relativeFilename The name of the log file to
        /// generate, sans the complete path.
        /// \param[in] _logCB Callback function, which is used to get log
        /// data.
        public: Log(LogRecord *_parent, const std::string &_relativeFilename,
                    boost::function<bool (std::ostringstream &)> _logCB);

        /// \brief Destructor
        public: virtual ~Log();

        /// \brief Start the log.
        /// \param[in] _path The complete path in which to put the log file.
        public: void Start(const boost::filesystem::path &_path);

        /// \brief Write data to disk.
        public: void Write();

        /// \brief Update the data buffer.
        public: void Update();

        /// \brief Clear the data buffer.
        public: void ClearBuffer();

        /// \brief Get the relative filename. This is the filename passed
        /// to the constructor.
        /// \return The relative filename.
        public: std::string GetRelativeFilename() const;

        /// \brief Pointer to the log record parent.
        public: LogRecord *parent;

        /// \brief Callback from which to get data.
        public: boost::function<bool (std::ostringstream &)> logCB;

        /// \brief Data buffer.
        public: std::string buffer;

        /// \brief The log file.
        public: std::ofstream logFile;

        /// \brief Relative log filename.
        public: std::string relativeFilename;

        private: boost::filesystem::path completePath;
      };
      /// \endcond

      /// \def Log_M
      /// \brief Map of names to logs.
      private: typedef std::map<std::string, Log*> Log_M;

      /// \brief All the log objects.
      private: Log_M logs;

      /// \brief Iterator used to update the log objects.
      private: Log_M::iterator updateIter;

      /// \brief Convenience iterator to the end of the log objects map.
      private: Log_M::iterator logsEnd;

      /// \brief Event connected to the World update.
      private: event::ConnectionPtr updateConnection;

      /// \brief True if logging is running.
      private: bool running;

      /// \brief Thread used to write data to disk.
      private: boost::thread *writeThread;

      /// \brief Mutext to protect writing.
      private: boost::mutex writeMutex;

      /// \brief Mutex to protect logging control.
      private: boost::mutex controlMutex;

      /// \brief Used by the write thread to know when data needs to be
      /// written to disk
      private: boost::condition_variable dataAvailableCondition;

      /// \brief The base pathname for all the logs.
      private: boost::filesystem::path logBasePath;

      /// \brief The complete pathname for all the logs.
      private: boost::filesystem::path logCompletePath;

      /// \brief Subdirectory for log files. This is appended to
      /// logBasePath.
      private: std::string logSubDir;

      /// \brief Encoding format for each chunk.
      private: std::string encoding;

      /// \brief True if initialized.
      private: bool initialized;

      /// \brief True to pause recording.
      private: bool paused;

      /// \brief This is a singleton
      private: friend class SingletonT<LogRecord>;
    };
    /// \}
  }
}
#endif
