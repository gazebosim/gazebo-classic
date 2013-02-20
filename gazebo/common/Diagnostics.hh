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
/* Desc: A diagnostic class
 * Author: Nate Koenig
 * Date: 2 Feb 2011
 */

#ifndef _DIAGNOSTICMANAGER_HH_
#define _DIAGNOSTICMANAGER_HH_

#include <map>
#include <string>

#include "gazebo/common/SingletonT.hh"
#include "gazebo/common/Timer.hh"

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common Common
    /// \{

    /// \brief Create an instance of common::DiagnosticManager
#ifdef ENABLE_DIAGNOSTICS
    #define DIAG_TIMER(name) \
    DiagnosticManager::Instance()->CreateTimer(name);

    #define DIAG_TIMER_LAP(name, prefix) \
    DiagnosticManager::Instance()->Lap(name, prefix);
#else
    #define DIAG_TIMER(name) ((void) 0)
    #define DIAG_TIMER_LAP(name, prefix) ((void)0)
#endif

    /// \class DiagnosticManager Diagnostics.hh common/common.hh
    /// \brief A diagnostic manager class
    class DiagnosticManager : public SingletonT<DiagnosticManager>
    {
      /// \brief Constructor
      private: DiagnosticManager();

      /// \brief Destructor
      private: virtual ~DiagnosticManager();

      /// \brief Create a new timer instance
      /// \param[in] _name Name of the timer.
      /// \return A pointer to the new diagnostic timer
      public: DiagnosticTimerPtr CreateTimer(const std::string &_name);

      //// \brief Output the current elapsed time of an active timer with
      /// a prefix string. This also resets the timer and keeps it running.
      /// \param[in] _name Name of the timer to access.
      /// \param[in] _prefix Informational string that is output with the
      /// elapsed time.
      public: void Lap(const std::string &_name, const std::string &_prefix);

      /// \brief A diagnostic timer has started
      /// \param[in] _timer The timer to start
      // public: void TimerStart(DiagnosticTimer *_timer);

      /// \brief A diagnostic timer has stopped
      /// \param[in] _time The timer to stop
      // public: void TimerStop(DiagnosticTimer *_timer);

      /// \brief Get the number of timers
      /// \return The number of timers
      public: int GetTimerCount() const;

      /// \brief Get the time of a timer instance
      /// \param[in] _index The index of a timer instance
      /// \return Time of the specified timer
      public: Time GetTime(int _index) const;

      /// \brief Get a time based on a label
      /// \param[in] _label Name of the timer instance
      /// \return Time of the specified timer
      public: Time GetTime(const std::string &_label) const;

      /// \brief Get a label for a timer
      /// \param[in] _index Index of a timer instance
      /// \return Label of the specified timer
      public: std::string GetLabel(int _index) const;

      private: typedef boost::weak_ptr<DiagnosticTimer> DiagnostTimerWeakPtr;
      private: typedef std::map<std::string, DiagnostTimerWeakPtr> TimerMap;

      /// \brief dictionary of timers index by name
      private: TimerMap timers;

      // Singleton implementation
      private: friend class SingletonT<DiagnosticManager>;
    };

    /// \class DiagnosticTimer Diagnostics.hh common/common.hh
    /// \brief A timer designed for diagnostics
    class DiagnosticTimer : public Timer
    {
      /// \brief Constructor
      /// \param[in] _name Name of the timer
      public: DiagnosticTimer(const std::string &_name) : Timer()
              {
                this->Start();
                this->name = _name;
                // this->diagManager->TimerStart(this);
                // event::Events::diagTimerStart(_timer->GetName());
              }

      /// \brief Destructor
      public: virtual ~DiagnosticTimer()
              {
                this->Stop();
                // event::Events::diagTimerStop(_timer->GetName());
                // this->diagManager->TimerStop(this);
              }

      /// \brief Get the name of the timer
      /// \return The name of timer
      public: inline const std::string GetName() const
              { return this->name; }

      /// \brief not used
      private: std::string name;

      /// \brief singleton instance
      // private: static DiagnosticManager *diagManager;
    };
    /// \}
  }
}
#endif
