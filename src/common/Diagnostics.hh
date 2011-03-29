/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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

#ifndef DIAGNOSTICMANAGER_HH
#define DIAGNOSTICMANAGER_HH

#include "common/SingletonT.hh"
#include "common/Timer.hh"

namespace gazebo
{
	namespace common
{

#define DIAG_TIMER(name) DiagnosticManager::Instance()->CreateTimer(name);

  class DiagnosticTimer;
  typedef boost::shared_ptr< DiagnosticTimer > DiagnosticTimerPtr;

  /// \brief A diagnostic manager class
  class DiagnosticManager : public SingletonT<DiagnosticManager>
  {
    /// \brief Constructor
    private: DiagnosticManager();

    /// \brief Destructor
    private: virtual ~DiagnosticManager();

    public: DiagnosticTimerPtr CreateTimer(const std::string &name);

    /// \brief A diagnostic timer has started
    public: void TimerStart(DiagnosticTimer *timer);

    /// \brief A diagnostic timer has stoped
    public: void TimerStop(DiagnosticTimer *timer);

    /// \brief Get the number of timers
    public: int GetTimerCount() const;

    /// \brief Get a specific time
    public: Time GetTime(int index) const;

    /// \brief Get a time based on a label
    public: Time GetTime(const std::string &label) const;

    /// \brief Get a label for a timer
    public: std::string GetLabel(int index) const;

    public: void SetEnabled(bool e) {this->enabled = e;}

    public: inline bool GetEnabled() const {return this->enabled;}

    private: bool enabled;

    private: std::map<std::string, Time> timers;

    //Singleton implementation
    private: friend class SingletonT<DiagnosticManager>;
  };

  /// \brief A timer designed for diagnostics
  class DiagnosticTimer : public Timer
  {
    /// \brief Constructor
    public: DiagnosticTimer(const std::string &name) : Timer() 
            {
              this->Start();
              this->name = name; 
              this->diagManager->TimerStart(this);
            }

    /// \brief Destructor
    public: virtual ~DiagnosticTimer() 
            { 
              this->diagManager->TimerStop(this);
            }

    public: inline const std::string GetName() const 
            { return this->name; }

    private: std::string name;
    private: static DiagnosticManager *diagManager;
  };

}
 
}
#endif
