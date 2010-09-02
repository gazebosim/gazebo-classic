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

#include "Simulator.hh"
#include "Timer.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
Timer::Timer(Type t)
{
  this->type = t;
  this->Start(); // auto start on construction, should not hurt
}
        
////////////////////////////////////////////////////////////////////////////////
/// Destructor
Timer::~Timer()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Start the timer
void Timer::Start()
{
  if (this->type == SIM_TIMER)
    this->start = Simulator::Instance()->GetSimTime();
  else
    this->start = Simulator::Instance()->GetWallTime();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the elapsed itme
Time Timer::GetElapsed() const
{
  Time currentTime;

  if (this->type == SIM_TIMER)
    currentTime = Simulator::Instance()->GetSimTime();
  else
    currentTime = Simulator::Instance()->GetWallTime();

  return currentTime - this->start;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the type of timer
Timer::Type Timer::GetType()
{
  return this->type;
}
