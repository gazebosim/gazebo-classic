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
#ifndef _GAZEBO_UTILS_DIAGNOSTICMANAGER_PRIVATE_HH_
#define _GAZEBO_UTILS_DIAGNOSTICMANAGER_PRIVATE_HH_

#include <fstream>
#include <string>
#include <boost/filesystem.hpp>

#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/common/Event.hh"
#include "gazebo/util/UtilTypes.hh"

namespace gazebo
{
  namespace util
  {
    /// \brief Private data for the DiagnosticManager class
    class DiagnosticManagerPrivate
    {
      /// \brief dictionary of timers index by name
      public: TimerMap timers;

      /// \brief Path in which to store timing logs.
      public: boost::filesystem::path logPath;

      /// \brief Node for publishing diagnostic data.
      public: transport::NodePtr node;

      /// \brief Publisher of diagnostic data.
      public: transport::PublisherPtr pub;

      /// \brief The message to output
      public: msgs::Diagnostics msg;

      /// \brief Pointer to the update event connection
      public: event::ConnectionPtr updateConnection;
    };

    /// \brief Private data for the DiagnosticTimer class
    class DiagnosticTimerPrivate
    {
      /// \brief Name of the timer.
      public: std::string name;

      /// \brief Log file.
      public: std::ofstream log;

      /// \brief Time of the previous lap.
      public: common::Time prevLap;
    };
  }
}
#endif
