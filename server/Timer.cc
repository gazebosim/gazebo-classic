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

#include "Simulator.hh"
#include "Timer.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
Timer::Timer(Type t)
{
  this->type = t;
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
