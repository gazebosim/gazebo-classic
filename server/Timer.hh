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
/* Desc: A timer class
 * Author: Nate Koenig
 * Date: 22 Nov 2009
 */

#ifndef TIMER_HH
#define TIMER_HH

#include "GazeboMessage.hh"
#include "Time.hh"

namespace gazebo
{
  /// \brief A timer class
  class Timer
  {
    public: enum Type {SIM_TIMER, REAL_TIMER};

    /// \brief Constructor
    /// \param t The type of timer (based on either the simulation or real
    ///          time)
    public: Timer(Type t=Timer::REAL_TIMER);
            
    /// \brief Destructor
    public: virtual ~Timer();

    /// \brief Start the timer
    public: void Start();

    /// \brief Get the elapsed itme
    public: Time GetElapsed() const;

    /// \brief Get the type of timer
    public: Type GetType();

    public: friend std::ostream &operator<<(std::ostream &out, 
                                            const gazebo::Timer &t)
            {
              out << t.GetElapsed();
              return out;
            }

    private: Time start;
    private: Type type;
  };

  /// \brief A timer designed for diagnostics
  class DiagnosticTimer : public Timer
  {
    /// \brief Constructor
    public: DiagnosticTimer(const std::string &name, int level = 6, 
                            Type t=Timer::REAL_TIMER) : Timer(t) 
            {
              this->name = name; 
              this->msgLevel = level; 
              this->Report("Start @ ");
            }

    /// \brief Destructor
    public: virtual ~DiagnosticTimer() 
            { 
              this->Report("Complete @ "); 
            }

    /// \brief Report a time
    public: void Report(const std::string msg)
            {
              gzmsg(this->msgLevel) << this->name << "["
                << msg << *this << "]\n";
            }

    private: int msgLevel;
    private: std::string name;

  };
}

#endif
