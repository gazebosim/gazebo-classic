/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef _GAZEBO_UTIL_LOGRECORD_PRIVATE_HH_
#define _GAZEBO_UTIL_LOGRECORD_PRIVATE_HH_

#include <map>
#include <string>
#include <thread>
#include <functional>
#include <condition_variable>
#include <boost/filesystem.hpp>

namespace gazebo
{
  namespace util
  {
    class LogRecord;

    /// \internal
    /// \brief Private data class for LogRecord.
    class LogRecordPrivate
    {
      /// \brief Destructor (makes style checker happy)
      public: virtual ~LogRecordPrivate() = default;

      /// \brief Log helper class
      public: class Log
      {
        /// \brief Constructor
        /// \param[in] _parent Pointer to the LogRecord parent.
        /// \param[in] _relativeFilename The name of the log file to
        /// generate, sans the complete path.
        /// \param[in] _logCB Callback function, which is used to get log
        /// data.
        public: Log(LogRecord *_parent, const std::string &_relativeFilename,
                    std::function<bool (std::ostringstream &)> _logCB);

        /// \brief Destructor
        public: virtual ~Log();

        /// \brief Start the log.
        /// \param[in] _path The complete path in which to put the log file.
        public: void Start(const boost::filesystem::path &_path);

        /// \brief Stop logging.
        public: void Stop();

        /// \brief Write data to disk.
        public: void Write();

        /// \brief Update the data buffer.
        /// \return The size of the data buffer.
        public: unsigned int Update();

        /// \brief Clear the data buffer.
        public: void ClearBuffer();

        /// \brief Get the byte size of the buffer.
        /// \return Buffer byte size.
        public: unsigned int BufferSize();

        /// \brief Get the relative filename. This is the filename passed
        /// to the constructor.
        /// \return The relative filename.
        public: std::string RelativeFilename() const;

        /// \brief Get the complete filename.
        /// \return The complete filename.
        public: std::string CompleteFilename() const;

        /// \brief Pointer to the log record parent.
        public: LogRecord *parent;

        /// \brief Callback from which to get data.
        public: std::function<bool (std::ostringstream &)> logCB;

        /// \brief Data buffer.
        public: std::string buffer;

        /// \brief The log file.
        public: std::ofstream logFile;

        /// \brief Relative log filename.
        public: std::string relativeFilename;

        /// \brief Complete file path.
        public: boost::filesystem::path completePath;
      };

      /// \def Log_M
      /// \brief Map of names to logs.
      public: typedef std::map<std::string, Log*> Log_M;

      /// \brief All the log objects.
      public: Log_M logs;

      /// \brief Iterator used to update the log objects.
      public: Log_M::iterator updateIter;

      /// \brief Convenience iterator to the end of the log objects map.
      public: Log_M::iterator logsEnd;

      /// \brief Condition used to start threads
      public: std::condition_variable startThreadCondition;

      /// \brief Condition used to trigger an update
      public: std::condition_variable updateCondition;

      /// \brief Used by the cleanupThread to wait for a cleanup signal.
      public: std::condition_variable cleanupCondition;

      /// \brief True if logging is running.
      public: bool running;

      /// \brief Thread used to write data to disk.
      public: std::thread *writeThread;

      /// \brief Thread used to update data.
      public: std::thread *updateThread;

      /// \brief Thread to cleanup log recording.
      public: std::thread cleanupThread;

      /// \brief Mutex to protect against parallel calls to Write()
      public: mutable std::mutex writeMutex;

      /// \brief Mutex to synchronize with RunWrite()
      public: mutable std::mutex runWriteMutex;

      /// \brief Mutex to synchronize with RunUpdate()
      public: mutable std::mutex updateMutex;

      /// \brief Mutex to protect logging control.
      public: std::mutex controlMutex;

      /// \brief Used by the write thread to know when data needs to be
      /// written to disk
      public: std::condition_variable dataAvailableCondition;

      /// \brief The base pathname for all the logs.
      public: boost::filesystem::path logBasePath;

      /// \brief The complete pathname for all the logs.
      public: boost::filesystem::path logCompletePath;

      /// \brief Subdirectory for log files. This is appended to
      /// logBasePath.
      public: std::string logSubDir;

      /// \brief Encoding format for each chunk.
      public: std::string encoding;

      /// \brief True if initialized.
      public: bool initialized;

      /// \brief True to pause recording.
      public: bool paused;

      /// \brief Used to indicate the first update callback.
      public: bool firstUpdate;

      /// \brief Flag used to stop the write thread.
      public: bool stopThread;

      /// \brief Start simulation time.
      public: common::Time startTime;

      /// \brief Current simulation time.
      public: common::Time currTime;

      /// \brief Transportation node.
      public: transport::NodePtr node;

      /// \brief Subscriber to log control messages.
      public: transport::SubscriberPtr logControlSub;

      /// \brief Publisher of log status messages.
      public: transport::PublisherPtr logStatusPub;

      /// \brief All the event connections.
      public: event::Connection_V connections;

      /// \brief Simulation pause state.
      public: bool pauseState;

      /// \brief True if the logger is ready to start, and the previous run
      /// has finished.
      public: bool readyToStart;
    };
    /// \}
  }
}
#endif
