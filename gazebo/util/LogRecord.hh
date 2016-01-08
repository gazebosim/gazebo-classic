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
#ifndef _GAZEBO_UTIL_LOGRECORD_HH_
#define _GAZEBO_UTIL_LOGRECORD_HH_

#include <fstream>
#include <string>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/common/SingletonT.hh"
#include "gazebo/util/system.hh"

#define GZ_LOG_VERSION "1.0"

namespace gazebo
{
  namespace util
  {
    // Forward declare private data class
    class LogRecordPrivate;

    /// addtogroup gazebo_util
    /// \{

    /// \class LogRecord LogRecord.hh util/util.hh
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
    class GZ_UTIL_VISIBLE LogRecord : public SingletonT<LogRecord>
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
                    std::function<bool (std::ostringstream &)> _logCallback);

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

      /// \brief Tell the recorder that an update should occur.
      public: void Notify();

      /// \brief Set whether logging should pause. A paused state means the
      /// log file is still open, but data is not written to it.
      /// \param[in] _paused True to pause data logging.
      /// \sa LogRecord::GetPaused
      public: void SetPaused(const bool _paused);

      /// \brief Get whether logging is paused.
      /// \return True if logging is paused.
      /// \sa LogRecord::SetPaused
      /// \deprecated See Paused() const
      public: bool GetPaused() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get whether logging is paused.
      /// \return True if logging is paused.
      /// \sa LogRecord::SetPaused
      public: bool Paused() const;

      /// \brief Get whether the logger is ready to start, which implies
      /// that any previous runs have finished.
      // \return True if logger is ready to start.
      public: bool IsReadyToStart() const;

      /// \brief Get whether logging is running.
      /// \return True if logging has been started.
      /// \deprecated See Running() const
      public: bool GetRunning() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get whether logging is running.
      /// \return True if logging has been started.
      public: bool Running() const;

      /// \brief Start the logger.
      /// \param[in] _encoding The type of encoding (txt, zlib, or bz2).
      /// \param[in] _path Path in which to store log files.
      public: bool Start(const std::string &_encoding="zlib",
                         const std::string &_path="");

      /// \brief Get the encoding used.
      /// \return Either [txt, zlib, or bz2], where txt is plain txt and bz2
      /// and zlib are compressed data with Base64 encoding.
      /// \deprecated See Encoding() const
      public: const std::string &GetEncoding() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the encoding used.
      /// \return Either [txt, zlib, or bz2], where txt is plain txt and bz2
      /// and zlib are compressed data with Base64 encoding.
      public: const std::string &Encoding() const;

      /// \brief Get the filename for a log object.
      /// \param[in] _name Name of the log object.
      /// \return Filename, empty string if not found.
      /// \deprecated See Filename(const std::string &) const
      public: std::string GetFilename(const std::string &_name = "") const
              GAZEBO_DEPRECATED(7.0);

      /// \brief Get the filename for a log object.
      /// \param[in] _name Name of the log object.
      /// \return Filename, empty string if not found.
      public: std::string Filename(const std::string &_name = "") const;

      /// \brief Get the file size for a log object.
      /// \param[in] _name Name of the log object.
      /// \return Size in bytes.
      /// \deprecated See FileSize(const std::string) const
      public: unsigned int GetFileSize(const std::string &_name = "") const
              GAZEBO_DEPRECATED(7.0);

      /// \brief Get the file size for a log object.
      /// \param[in] _name Name of the log object.
      /// \return Size in bytes.
      public: unsigned int FileSize(const std::string &_name = "") const;

      /// \brief Set the base path.
      /// \param[in] _path Path to the new logging location.
      public: void SetBasePath(const std::string &_path);

      /// \brief Get the base path for a log recording.
      /// \return Path for log recording.
      /// \deprecated See BasePath() const
      public: std::string GetBasePath() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the base path for a log recording.
      /// \return Path for log recording.
      public: std::string BasePath() const;

      /// \brief Get the run time in sim time.
      /// \return Run sim time.
      /// \deprecated See RunTime() const
      public: common::Time GetRunTime() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the run time in sim time.
      /// \return Run sim time.
      public: common::Time RunTime() const;

      /// \brief Finialize, and shutdown.
      public: void Fini();

      /// \brief Return true if an Update has not yet been completed.
      /// \return True if an Update has not yet been completed.
      /// \deprecated See FirstUpdate()
      public: bool GetFirstUpdate() const GAZEBO_DEPRECATED(7.0);

      /// \brief Return true if an Update has not yet been completed.
      /// \return True if an Update has not yet been completed.
      public: bool FirstUpdate() const;

      /// \brief Write all logs.
      /// \param[in] _force True to skip waiting on dataAvailableCondition.
      public: void Write(const bool _force = false);

      /// \brief Get the size of the buffer.
      /// \return Size of the buffer, in bytes.
      /// \deprecated See BufferSize() const
      public: unsigned int GetBufferSize() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the size of the buffer.
      /// \return Size of the buffer, in bytes.
      public: unsigned int BufferSize() const;

      /// \brief Update the log files
      ///
      /// Captures the current state of all registered entities, and outputs
      /// the data to their respective log files.
      private: void Update();

      /// \brief Function used by the update thread.
      private: void RunUpdate();

      /// \brief Run the Write loop.
      private: void RunWrite();

      /// \brief Clear and delete the log buffers.
      private: void ClearLogs();

      /// \brief Publish log status message.
      private: void PublishLogStatus();

      /// \brief Called when a log control message is received.
      /// \param[in] _data The log control message.
      private: void OnLogControl(ConstLogControlPtr &_data);

      /// \brief Cleanup log recording. A thread uses this function, you
      /// should never call this function directly. Use the Stop() function
      /// to trigger a cleanup.
      private: void Cleanup();

      /// \brief Used to get the simulation pause state.
      private: void OnPause(const bool _pause);

      /// \brief This is a singleton
      private: friend class SingletonT<LogRecord>;

      /// \internal
      /// \brief Private data pointer.
      private: std::unique_ptr<LogRecordPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
