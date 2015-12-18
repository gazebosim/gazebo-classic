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
#ifndef _GAZEBO_UTIL_LOGPLAY_HH_
#define _GAZEBO_UTIL_LOGPLAY_HH_

#include <tinyxml2.h>
#include <string>

#include "gazebo/common/SingletonT.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace util
  {
    // Forward declare private data class
    class LogPlayPrivate;

    /// \addtogroup gazebo_physics
    /// \{

    /// \class Logplay Logplay.hh util/util.hh
    /// \brief Open and playback log files that were recorded using LogRecord.
    ///
    /// Use Logplay to open a log file (Logplay::Open), and access the
    /// recorded state information. Iterators are available to step through
    /// the state information. It is also possible to replay the data in a
    /// World using the Play functions. Replay involves reading and applying
    /// state information to a World.
    ///
    /// \sa LogRecord, State
    class GZ_UTIL_VISIBLE LogPlay : public SingletonT<LogPlay>
    {
      /// \brief Constructor
      private: LogPlay();

      /// \brief Destructor
      private: virtual ~LogPlay();

      /// \brief Open a log file for reading
      ///
      /// Open a log file that was previously recorded.
      /// \param[in] _logFile The file to load
      /// \throws Exception When the log file does not exist, is a directory
      /// instead of a regular file, or Gazebo was unable to parse it.
      public: void Open(const std::string &_logFile);

      /// \brief Return true if a file is open.
      /// \return True if a log file is open.
      public: bool IsOpen() const;

      /// \brief Get the log version number of the open log file.
      /// \return The log version of the open log file. Empty string if
      /// a log file is not open.
      /// \deprecated See LogVersion() const
      public: std::string GetLogVersion() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the log version number of the open log file.
      /// \return The log version of the open log file. Empty string if
      /// a log file is not open.
      public: std::string LogVersion() const;

      /// \brief Get the Gazebo version number of the open log file.
      /// \return The Gazebo version of the open log file. Empty string if
      /// a log file is not open.
      /// \deprecated See GazeboVersion()
      public: std::string GetGazeboVersion() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the Gazebo version number of the open log file.
      /// \return The Gazebo version of the open log file. Empty string if
      /// a log file is not open.
      public: std::string GazeboVersion() const;

      /// \brief Get the random number seed of the open log file.
      /// \return The random number seed the open log file. The current
      /// random number seed, as defined in ignition::math::Rand::Seed.
      /// \deprecated See RandSeed() const
      public: uint32_t GetRandSeed() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the random number seed of the open log file.
      /// \return The random number seed the open log file. The current
      /// random number seed, as defined in ignition::math::Rand::Seed.
      public: uint32_t RandSeed() const;

      /// \brief Get the log start time of the open log file.
      /// \return Start time of the log.
      /// \deprecated See LogStartTime() const
      public: common::Time GetLogStartTime() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the log start time of the open log file.
      /// \return Start time of the log.
      public: common::Time LogStartTime() const;

      /// \brief Get the log end time of the open log file.
      /// \return End time of the log.
      /// \deprecated See LogEndTime() const
      public: common::Time GetLogEndTime() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the log end time of the open log file.
      /// \return End time of the log.
      public: common::Time LogEndTime() const;

      /// \brief Get the name of the log file.
      /// \return The filename.
      /// \deprecated See Filename() const;
      public: std::string GetFilename() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the name of the log file.
      /// \return The filename.
      public: std::string Filename() const;

      /// \brief Get the full path of the log file.
      /// \return The full path of the log file.
      /// \deprecated See FullPathFilename()
      public: std::string GetFullPathFilename() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the full path of the log file.
      /// \return The full path of the log file.
      public: std::string FullPathFilename() const;

      /// \brief Get the size of the log file.
      /// \return The size of the file in bytes.
      /// \deprecated See FileSize() const
      public: uintmax_t GetFileSize() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the size of the log file.
      /// \return The size of the file in bytes.
      public: uintmax_t FileSize() const;

      /// \brief Step through the open log file.
      /// \param[out] _data Data from next entry in the log file.
      public: bool Step(std::string &_data);

      /// \brief Step through the open log file backwards.
      /// \param[out] _data Data from next entry in the log file.
      public: bool StepBack(std::string &_data);

      /// \brief Step through the open log file.
      /// \param[in] _step Number of samples to step (forward or backwards).
      /// \param[out] _data Data from next entry in the log file.
      public: bool Step(const int _step, std::string &_data);

      /// \brief Jump to the closest sample that has its simulation time lower
      /// than the time specified as a parameter.
      /// \param[in] _time Target simulation time.
      /// \return True if operation succeed or false otherwise.
      public: bool Seek(const common::Time &_time);

      /// \brief Jump to the beginning of the log file. The next step() call
      /// will return the first data "chunk".
      /// \return True If the function succeed or false otherwise.
      public: bool Rewind();

      /// \brief Jump to the end of the log file.
      /// \return True If the function succeed or false otherwise.
      public: bool Forward();

      /// \brief Get the number of chunks (steps) in the open log file.
      /// \return The number of recorded states in the log file.
      /// \deprecated See ChunkCount() const
      public: unsigned int GetChunkCount() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the number of chunks (steps) in the open log file.
      /// \return The number of recorded states in the log file.
      public: unsigned int ChunkCount() const;

      /// \brief Get data for a particular chunk index.
      /// \param[in] _index Index of the chunk.
      /// \param[out] _data Storage for the chunk's data.
      /// \return True if the _index was valid.
      /// \deprecated See GetChunk(unsigned int, std::string &) const
      public: bool GetChunk(unsigned int _index, std::string &_data)
              GAZEBO_DEPRECATED(7.0);

      /// \brief Get data for a particular chunk index.
      /// \param[in] _index Index of the chunk.
      /// \param[out] _data Storage for the chunk's data.
      /// \return True if the _index was valid.
      public: bool Chunk(const unsigned int _index, std::string &_data) const;

      /// \brief Get the type of encoding used for current chunck in the
      /// open log file.
      /// \return The type of encoding. An empty string will be returned if
      /// LogPlay::Step has not been called at least once.
      /// \deprecated See Encoding() const
      public: std::string GetEncoding() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the type of encoding used for current chunck in the
      /// open log file.
      /// \return The type of encoding. An empty string will be returned if
      /// LogPlay::Step has not been called at least once.
      public: std::string Encoding() const;

      /// \brief Get the header that was read from a log file. Should call
      /// LogPlay::Open first.
      /// \return Header of the open log file.
      /// \deprecated See Header() const
      public: std::string GetHeader() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the header that was read from a log file. Should call
      /// LogPlay::Open first.
      /// \return Header of the open log file.
      public: std::string Header() const;

      /// \brief Get the initial simulation iterations from a log file.
      /// \return Initial simulation iteration contained in the log file.
      /// \deprecated See InitialIterations() const
      public: uint64_t GetInitialIterations() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the initial simulation iterations from a log file.
      /// \return Initial simulation iteration contained in the log file.
      public: uint64_t InitialIterations() const;

      /// \brief Return if the log file contains the <iterations> tag. Old log
      /// files may not have the <iterations> tag.
      /// \return True if <iterations> was found in the log file or
      /// false otherwise.
      public: bool HasIterations() const;

      /// \brief Helper function to get chunk data from XML.
      /// \param[in] _xml Pointer to an xml block that has state data.
      /// \param[out] _data Storage for the chunk's data.
      /// \return True if the chunk was successfully parsed.
      private: bool ChunkData(tinyxml2::XMLElement *_xml,
                              std::string &_data) const;

      /// \brief Read the header from the log file.
      private: void ReadHeader();

      /// \brief Update the internal variables that keep track of the times
      /// where the log started and finished (simulation time).
      private: void ReadLogTimes();

      /// \brief Update the internal variable that keeps track of the initial
      /// "iterations" value.
      /// \return True when the operation succeed or false otherwise
      /// (e.g.: if the <iterations> elements are not found).
      private: bool ReadIterations();

      /// \brief If possible, jump to the next chunk.
      /// \return True if the operation succeed or false if there were no more
      /// chunks after the current one.
      private: bool NextChunk();

      /// \brief If possible, jump to the previous chunk.
      /// \return True if the operation succeed or false if there were no more
      /// chunks before the current one.
      private: bool PrevChunk();

      /// \internal
      /// \brief Private data pointer
      private: std::unique_ptr<LogPlayPrivate> dataPtr;

      /// \brief This is a singleton
      private: friend class SingletonT<LogPlay>;
    };
    /// \}
  }
}

#endif
