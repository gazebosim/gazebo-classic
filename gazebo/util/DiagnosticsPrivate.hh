/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
#include "gazebo/util/Diagnostics.hh"
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

      /// \brief Listen to control messages
      public: transport::SubscriberPtr controlSub;

      /// \brief The message to output
      public: msgs::Diagnostics msg;

      /// \brief Pointer to the update event connection
      public: event::ConnectionPtr updateConnection;

      /// \brief Log file for variables.
      public: std::ofstream varLog;

      /// \brief True if diagnostics are enabled.
      public: bool enabled;
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

    /// \brief A no-op function used by Diagnostics when it is disabled.
    static inline void _DiagnosticManager_Noop1(const std::string &/*_name*/) {}

    /// \brief A no-op function used by Diagnostics when it is disabled.
    static inline void _DiagnosticManager_Noop2(const std::string &/*_name*/,
        const std::string &/*_prefix*/) {}

    /// \brief A no-op function used by Diagnostics when it is disabled.
    static inline void _DiagnosticManager_Noop3(const std::string &/*_name*/,
        double /*_value*/) {}


    /// \brief Function used to start a timer.
    /// \param[in] _name Name of the timer to start
    static inline void _DiagnosticManager_Start(const std::string &_name)
    {
      gazebo::util::DiagnosticManager::Instance()->StartTimer(_name);
    }

    /// \brief Function used to stop a timer.
    /// \param[in] _name Name of the timer to stop
    static inline void _DiagnosticManager_Stop(const std::string &_name)
    {
      gazebo::util::DiagnosticManager::Instance()->StopTimer(_name);
    }

    /// \brief Function used to produce a lap time.
    /// \param[in] _name Name of the timer
    /// \param[in] _prefix String to accompany the lap time.
    static inline void _DiagnosticManager_Lap(const std::string &_name,
                                              const std::string &_prefix)
    {
      gazebo::util::DiagnosticManager::Instance()->Lap(_name, _prefix);
    }

    /// \brief Function used to add a variable to diagnostics.
    /// \param[in] _name Name associated with the variable.
    /// \param[in] _value A value to add
    static inline void _DiagnosticManager_Variable(const std::string &_name,
                                                   double _value)
    {
      gazebo::util::DiagnosticManager::Instance()->Variable(_name, _value);
    }

    /// \brief Function used to add a marker to diagnostics.
    /// \param[in] _name Name of the marker.
    static inline void _DiagnosticManager_Marker(const std::string &_name)
    {
      gazebo::util::DiagnosticManager::Instance()->Marker(_name);
    }

    /// \brief Function pointer to start a timer.
    void (*_diagStartPtr)(const std::string &_name) = &_DiagnosticManager_Noop1;

    /// \brief Function pointer to lap a timer.
    void (*_diagLapPtr)(const std::string &_name, const std::string &_prefix) =
      &_DiagnosticManager_Noop2;

    /// \brief Function pointer to stop a timer.
    void (*_diagStopPtr)(const std::string &_name) = &_DiagnosticManager_Noop1;

    /// \brief Function pointer a variable diagnostic information.
    void (*_diagVariablePtr)(const std::string &_name, double _var) =
      &_DiagnosticManager_Noop3;

    /// \brief Function pointer for a marker.
    void (*_diagMarkerPtr)(const std::string &_name) =
      &_DiagnosticManager_Noop1;
  }
}
#endif
