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
/* Desc: A diagnostic class
 * Author: Nate Koenig
 * Date: 2 Feb 2011
 */

#ifndef _DIAGNOSTICMANAGER_HH_
#define _DIAGNOSTICMANAGER_HH_

#include <boost/unordered_map.hpp>
#include <string>
#include <boost/filesystem.hpp>

#include "gazebo/gazebo_config.h"

#include "gazebo/transport/TransportTypes.hh"

#include "gazebo/msgs/msgs.hh"

#include "gazebo/common/UpdateInfo.hh"
#include "gazebo/common/SingletonT.hh"
#include "gazebo/common/Timer.hh"

#include "gazebo/util/UtilTypes.hh"

namespace gazebo
{
  namespace util
  {
    /// \addtogroup gazebo_util Utility
    /// \{

#ifdef ENABLE_DIAGNOSTICS
    /// \brief Start a diagnostic timer. Make sure to run DIAG_TIMER_STOP to
    /// stop the timer.
    /// \param[in] _name Name of the timer to start.
    #define DIAG_TIMER_START(_name) \
    gazebo::util::DiagnosticManager::Instance()->StartTimer(_name);

    /// \brief Output a lap time annotated with a prefix string. A lap is
    /// the time from last call to DIAG_TIMER_LAP or DIAG_TIMER_START, which
    /// every occured last.
    /// \param[in] _name Name of the timer.
    /// \param[in] _prefix String for annotation.
    #define DIAG_TIMER_LAP(_name, _prefix) \
    gazebo::util::DiagnosticManager::Instance()->Lap(_name, _prefix);

    /// \brief Stop a diagnostic timer.
    /// \param[in] name Name of the timer to stop
    #define DIAG_TIMER_STOP(_name) \
    gazebo::util::DiagnosticManager::Instance()->StopTimer(_name);
#else
    #define DIAG_TIMER_START(_name) ((void) 0)
    #define DIAG_TIMER_LAP(_name, _prefix) ((void)0)
    #define DIAG_TIMER_STOP(_name) ((void) 0)
#endif

    /// \class DiagnosticManager Diagnostics.hh util/util.hh
    /// \brief A diagnostic manager class
    class DiagnosticManager : public SingletonT<DiagnosticManager>
    {
      /// \brief Constructor
      private: DiagnosticManager();

      /// \brief Destructor
      private: virtual ~DiagnosticManager();

      /// \brief Initialize to report diagnostics about a world.
      /// \param[in] _worldName Name of the world.
      public: void Init(const std::string &_worldName);

      /// \brief Start a new timer instance
      /// \param[in] _name Name of the timer.
      /// \return A pointer to the new diagnostic timer
      public: void StartTimer(const std::string &_name);

      /// \brief Stop a currently running timer.
      /// \param[in] _name Name of the timer to stop.
      public: void StopTimer(const std::string &_name);

      //// \brief Output the current elapsed time of an active timer with
      /// a prefix string. This also resets the timer and keeps it running.
      /// \param[in] _name Name of the timer to access.
      /// \param[in] _prefix Informational string that is output with the
      /// elapsed time.
      public: void Lap(const std::string &_name, const std::string &_prefix);

      /// \brief Get the number of timers
      /// \return The number of timers
      public: int GetTimerCount() const;

      /// \brief Get the time of a timer instance
      /// \param[in] _index The index of a timer instance
      /// \return Time of the specified timer
      public: common::Time GetTime(int _index) const;

      /// \brief Get a time based on a label
      /// \param[in] _label Name of the timer instance
      /// \return Time of the specified timer
      public: common::Time GetTime(const std::string &_label) const;

      /// \brief Get a label for a timer
      /// \param[in] _index Index of a timer instance
      /// \return Label of the specified timer
      public: std::string GetLabel(int _index) const;

      /// \brief Get the path in which logs are stored.
      /// \return The path in which logs are stored.
      public: boost::filesystem::path GetLogPath() const;

      /// \brief Publishes diagnostic information.
      /// \param[in] _info World update information.
      private: void Update(const common::UpdateInfo &_info);

      /// \brief Add a time for publication.
      /// \param[in] _name Name of the diagnostic time.
      /// \param[in] _wallTime Wall clock time stamp.
      /// \param[in] _elapsedTime Elapsed time, this is the time
      /// measurement.
      private: void AddTime(const std::string &_name, common::Time &_wallTime,
                   common::Time &_elapsedtime);

      /// \brief Map of all the active timers.
      private: typedef boost::unordered_map<std::string, DiagnosticTimerPtr>
               TimerMap;

      /// \brief dictionary of timers index by name
      private: TimerMap timers;

      /// \brief Path in which to store timing logs.
      private: boost::filesystem::path logPath;

      /// \brief Node for publishing diagnostic data.
      private: transport::NodePtr node;

      /// \brief Publisher of diagnostic data.
      private: transport::PublisherPtr pub;

      /// \brief The message to output
      private: msgs::Diagnostics msg;

      /// \brief Pointer to the update event connection
      private: event::ConnectionPtr updateConnection;

      // Singleton implementation
      private: friend class SingletonT<DiagnosticManager>;

      /// \brief Give DiagnosticTimer special rights.
      private: friend class DiagnosticTimer;
    };

    /// \class DiagnosticTimer Diagnostics.hh util/util.hh
    /// \brief A timer designed for diagnostics
    class DiagnosticTimer : public common::Timer
    {
      /// \brief Constructor
      /// \param[in] _name Name of the timer
      public: DiagnosticTimer(const std::string &_name);

      /// \brief Destructor
      public: virtual ~DiagnosticTimer();

      /// \brief Output a lap time.
      /// \param[in] _prefix Annotation to output with the elapsed time.
      public: void Lap(const std::string &_prefix);

      // Documentation inherited
      public: virtual void Start();

      // Documentation inherited
      public: virtual void Stop();

      /// \brief Get the name of the timer
      /// \return The name of timer
      public: inline const std::string GetName() const
              { return this->name; }

      /// \brief Name of the timer.
      private: std::string name;

      /// \brief Log file.
      private: std::ofstream log;

      /// \brief Time of the previous lap.
      private: common::Time prevLap;
    };
    /// \}
  }
}
#endif
