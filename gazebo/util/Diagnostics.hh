/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_UTIL_DIAGNOSTICMANAGER_HH_
#define _GAZEBO_UTIL_DIAGNOSTICMANAGER_HH_

#include <string>
#include <boost/filesystem.hpp>

#include "gazebo/gazebo_config.h"

#include "gazebo/common/UpdateInfo.hh"
#include "gazebo/common/SingletonT.hh"
#include "gazebo/common/Timer.hh"

#include "gazebo/util/UtilTypes.hh"
#include "gazebo/util/system.hh"

#include "gazebo/transport/transport.hh"

#include "gazebo/msgs/MessageTypes.hh"

namespace gazebo
{
  namespace util
  {
    // Forward declare private data class
    class DiagnosticManagerPrivate;

    // Forward declare private data class
    class DiagnosticTimerPrivate;

    /// \addtogroup gazebo_util Utility
    /// \brief Useful utility functions that typically rely on the transport
    /// library.
    /// \{

#define ENABLE_DIAGNOSTICS
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

    /// \brief Add an a arbitrary variable to diagnostics.
    /// \param[in] _name Name associated with the variable.
    /// \param[in] _value Value of the variable. Value must be a double.
    #define DIAG_VARIABLE(_name, _value) \
    gazebo::util::DiagnosticManager::Instance()->Variable(_name, _value);

    /// \brief Add a marker at the current time.
    /// \param[in] _name Name of the marker
    #define DIAG_MARKER(_name) \
    gazebo::util::DiagnosticManager::Instance()->Marker(_name);

    /// \brief Check if diagnostics is active
    /// \return true if active, false if not
    #define DIAG_ENABLED() \
    gazebo::util::DiagnosticManager::Instance()->GetEnabled()
#else
    #define DIAG_TIMER_START(_name) ((void) 0)
    #define DIAG_TIMER_LAP(_name, _prefix) ((void)0)
    #define DIAG_TIMER_STOP(_name) ((void) 0)
    #define DIAG_VARIABLE(_name, _value) ((void) 0)
    #define DIAG_MARKER(_name) ((void) 0)
    #define DIAG_ENABLED() false
#endif

    /// \class DiagnosticManager Diagnostics.hh util/util.hh
    /// \brief A diagnostic manager class
    class GZ_UTIL_VISIBLE DiagnosticManager :
      public SingletonT<DiagnosticManager>
    {
      /// \brief Constructor
      private: DiagnosticManager();

      /// \brief Destructor
      private: virtual ~DiagnosticManager();

      /// \brief Initialize to report diagnostics about a world.
      /// \param[in] _worldName Name of the world.
      public: void Init(const std::string &_worldName);

      /// \brief Shutdown diagnostics.
      public: void Fini();

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

      /// \brief Add an an arbitrary variable to diagnostics.
      /// \param[in] _name Name associated with the variable.
      /// \param[in] _value Value of the variable.
      public: void Variable(const std::string &_name, double _value);

      /// \brief Add an a marker to diagnostics.
      /// \param[in] _name Name of the marker.
      public: void Marker(const std::string &_name);

      /// \brief Get the number of timers
      /// \return The number of timers
      /// \deprecated See TimerCount() const
      public: int GetTimerCount() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the number of timers
      /// \return The number of timers
      public: int TimerCount() const;

      /// \brief Get the time of a timer instance
      /// \param[in] _index The index of a timer instance
      /// \return Time of the specified timer
      /// \deprecated See Time(const int) const;
      public: common::Time GetTime(int _index) const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the time of a timer instance
      /// \param[in] _index The index of a timer instance
      /// \return Time of the specified timer
      public: common::Time Time(const int _index) const;

      /// \brief Get a time based on a label
      /// \param[in] _label Name of the timer instance
      /// \return Time of the specified timer
      /// \deprecated See Time(const std::string &_label) const
      public: common::Time GetTime(const std::string &_label) const
              GAZEBO_DEPRECATED(7.0);

      /// \brief Get a time based on a label
      /// \param[in] _label Name of the timer instance
      /// \return Time of the specified timer
      public: common::Time Time(const std::string &_label) const;

      /// \brief Get a label for a timer
      /// \param[in] _index Index of a timer instance
      /// \return Label of the specified timer
      /// \deprecated See Label(const int) const
      public: std::string GetLabel(int _index) const GAZEBO_DEPRECATED(7.0);

      /// \brief Get a label for a timer
      /// \param[in] _index Index of a timer instance
      /// \return Label of the specified timer
      public: std::string Label(const int _index) const;

      /// \brief Get the path in which logs are stored.
      /// \return The path in which logs are stored.
      /// \deprecated See LogPath() const
      public: boost::filesystem::path GetLogPath() const GAZEBO_DEPRECATED(7.0);

      /// \brief Returns true if diagnostics is active.
      /// \return True when generating diagnostic information.
      public: bool GetEnabled() const;

      /// \brief Set whether diagnostics is active.
      /// \param[in] _enabled True to enable diagnostics.
      public: void SetEnabled(bool _enabled);

      /// \brief Get the path in which logs are stored.
      /// \return The path in which logs are stored.
      public: boost::filesystem::path LogPath() const;

      /// \brief Publishes diagnostic information.
      /// \param[in] _info World update information.
      private: void Update(const common::UpdateInfo &_info);

      /// \brief Add a time for publication.
      /// \param[in] _name Name of the diagnostic time.
      /// \param[in] _wallTime Wall clock time stamp.
      /// \param[in] _elapsedTime Elapsed time, this is the time
      /// measurement.
      private: void AddTime(const std::string &_name,
                   const common::Time &_wallTime,
                   const common::Time &_elapsedtime);

      /// \brief Recive control messages.
      private: void OnControl(ConstDiagnosticControlPtr &_msg);

      // Singleton implementation
      private: friend class SingletonT<DiagnosticManager>;

      /// \brief Give DiagnosticTimer special rights.
      private: friend class DiagnosticTimer;

      /// \internal
      /// \brief Private data pointer
      private: std::unique_ptr<DiagnosticManagerPrivate> dataPtr;
    };

    /// \class DiagnosticTimer Diagnostics.hh util/util.hh
    /// \brief A timer designed for diagnostics
    class GZ_UTIL_VISIBLE DiagnosticTimer : public common::Timer
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
      /// \deprecated See Name() const
      public: const std::string GetName() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the name of the timer
      /// \return The name of timer
      public: const std::string Name() const;

      /// \internal
      /// \brief Private data pointer
      private: std::unique_ptr<DiagnosticTimerPrivate> dataPtr;
    };
    /// \}

    /// \cond
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
    /// \endcond
  }
}
#endif
