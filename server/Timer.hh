/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: A timer class
 * Author: Nate Koenig
 * Date: 22 Nov 2009
 */

#ifndef TIMER_HH
#define TIMER_HH

#include "gazebo_config.h"
#include "GazeboMessage.hh"
#include "Time.hh"
#include <iostream>
#include <iomanip>

#ifdef ENABLE_TIMERS
#define DIAGNOSTICTIMER(x) DiagnosticTimer x
#else
#define DIAGNOSTICTIMER(x) ((void)0)
#endif

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
              out << std::fixed << std::setprecision(6) << t.GetElapsed();
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
              //this->Report("Start @ ");
            }

    /// \brief Destructor
    public: virtual ~DiagnosticTimer() 
            { 
              this->Report("Duration: "); 
            }

    /// \brief Report a time
    public: void Report(const std::string msg)
            {
              //switching to printf so messages do not get broken up into interlaced pieces on multiple threads
              //(gazebo::GazeboMessage::Instance()->Msg(this->msgLevel)) << this->name << "[" << msg << *this << "]\n";
              if (gazebo::GazeboMessage::Instance()->GetVerbose() >= this->msgLevel)
              {
                std::ostringstream stream;
                stream << this->name << "[" << msg << *this << "]\n";
                printf("%s",stream.str().c_str());
              }
            }

    private: int msgLevel;
    private: std::string name;

  };
}

#endif
