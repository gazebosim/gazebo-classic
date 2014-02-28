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

#ifndef _LOGPLAY_HH_
#define _LOGPLAY_HH_

#include <tinyxml.h>

#include <list>
#include <string>
#include <fstream>

#include "gazebo/common/SingletonT.hh"

namespace gazebo
{
  namespace util
  {
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
    class LogPlay : public SingletonT<LogPlay>
    {
      /// \brief Constructor
      private: LogPlay();

      /// \brief Destructor
      private: virtual ~LogPlay();

      /// \brief Open a log file for reading
      ///
      /// Open a log file that was previously recorded.
      /// \param[in] _logFile The file to load
      /// \throws Exception
      public: void Open(const std::string &_logFile);

      /// \brief Return true if a file is open.
      /// \return True if a log file is open.
      public: bool IsOpen() const;

      /// \brief Get the log version number of the open log file.
      /// \return The log version of the open log file. Empty string if
      /// a log file is not open.
      public: std::string GetLogVersion() const;

      /// \brief Get the Gazebo version number of the open log file.
      /// \return The Gazebo version of the open log file. Empty string if
      /// a log file is not open.
      public: std::string GetGazeboVersion() const;

      /// \brief Get the random number seed of the open log file.
      /// \return The random number seed the open log file. The current
      /// random number seed, as defined in math::Rand::GetSeed.
      public: uint32_t GetRandSeed() const;

      /// \brief Step through the open log file.
      /// \param[out] _data Data from next entry in the log file.
      public: bool Step(std::string &_data);

      /// \brief Get the number of chunks (steps) in the open log file.
      /// \return The number of recorded states in the log file.
      public: unsigned int GetChunkCount() const;

      /// \brief Get data for a particular chunk index.
      /// \param[in] _index Index of the chunk.
      /// \param[out] _data Storage for the chunk's data.
      /// \return True if the _index was valid.
      public: bool GetChunk(unsigned int _index, std::string &_data);

      /// \brief Get the type of encoding used for current chunck in the
      /// open log file.
      /// \return The type of encoding. An empty string will be returned if
      /// LogPlay::Step has not been called at least once.
      public: std::string GetEncoding() const;

      /// \brief Get the header that was read from a log file. Should call
      /// LogPlay::Open first.
      /// \return Header of the open log file.
      public: std::string GetHeader() const;

      /// \brief Helper function to get chunk data from XML.
      /// \param[in] _xml Pointer to an xml block that has state data.
      /// \param[out] _data Storage for the chunk's data.
      /// \return True if the chunk was successfully parsed.
      private: bool GetChunkData(TiXmlElement *_xml, std::string &_data);

      /// \brief Read the header from the log file.
      private: void ReadHeader();

      /// \brief The XML document of the log file.
      private: TiXmlDocument xmlDoc;

      /// \brief Start of the log.
      private: TiXmlElement *logStartXml;

      /// \brief Current position in the log file.
      private: TiXmlElement *logCurrXml;

      /// \brief Name of the log file.
      private: std::string filename;

      /// \brief The version of the Gazebo logger used to create the open
      /// log file.
      private: std::string logVersion;

      /// \brief The version of Gazebo used to create the open log file.
      private: std::string gazeboVersion;

      /// \brief The random number seed recorded in the open log file.
      private: uint32_t randSeed;

      /// \brief The encoding for the current chunk in the log file.
      private: std::string encoding;

      private: std::string currentChunk;

      /// \brief This is a singleton
      private: friend class SingletonT<LogPlay>;
    };
    /// \}
  }
}

#endif
