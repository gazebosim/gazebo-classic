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
#ifndef _GAZEBO_UTIL_LOGPLAY_PRIVATE_HH_
#define _GAZEBO_UTIL_LOGPLAY_PRIVATE_HH_

#include <tinyxml2.h>
#include <mutex>
#include <string>

#include "gazebo/common/Time.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace util
  {
    /// \internal
    /// \brief Private data for log play
    class LogPlayPrivate
    {
      /// \brief Max number of chunks to inspect when looking for XML elements.
      public: const unsigned int kNumChunksToTry = 2u;

      /// \brief XML tag delimiting the beginning of a frame.
      public: const std::string kStartFrame = "<sdf ";

      /// \brief XML tag delimiting the end of a frame.
      public: const std::string kEndFrame   = "</sdf>";

      /// \brief XML tag delimiting the beginning of a simulation time element.
      public: const std::string kStartTime = "<sim_time>";

      /// \brief XML tag delimiting the end of a simulation time element.
      public: const std::string kEndTime = "</sim_time>";

      /// \brief The XML document of the log file.
      public: tinyxml2::XMLDocument xmlDoc;

      /// \brief Start of the log.
      public: tinyxml2::XMLElement *logStartXml;

      /// \brief Current position in the log file.
      public: tinyxml2::XMLElement *logCurrXml;

      /// \brief Name of the log file.
      public: std::string filename;

      /// \brief The version of the Gazebo logger used to create the open
      /// log file.
      public: std::string logVersion;

      /// \brief The version of Gazebo used to create the open log file.
      public: std::string gazeboVersion;

      /// \brief The random number seed recorded in the open log file.
      public: uint32_t randSeed;

      /// \brief Log start time (simulation time).
      public: common::Time logStartTime;

      /// \brief Log end time (simulation time).
      public: common::Time logEndTime;

      /// \brief The encoding for the current chunk in the log file.
      public: std::string encoding;

      /// \brief This is the chunk where the current frame is contained.
      public: std::string currentChunk;

      /// \brief The current chunk might contain multiple frames.
      /// This variable points to the beginning of the last frame dispatched.
      public: size_t start;

      /// \brief The current chunk might contain multiple frames.
      /// This variable points to the end of the last frame dispatched.
      public: size_t end;

      /// \brief Initial simulation iteration contained in the log file.
      public: uint64_t initialIterations;

      /// \brief True if <iterations> is found in the log file. Old log versions
      /// may not include this tag in the log files.
      public: bool iterationsFound;

      /// \brief A mutex to avoid race conditions.
      public: std::mutex mutex;
    };
  }
}
#endif
