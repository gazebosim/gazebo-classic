/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
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

#include "gazebo/common/Timer.hh"

using namespace gazebo;
using namespace common;

//////////////////////////////////////////////////
Timer::Timer()
  : reset(true), running(false), countdown(false)
{
}

//////////////////////////////////////////////////
Timer::Timer(const Time &_maxTime, const bool _countdown)
  : reset(true), running(false), countdown(_countdown), maxTime(_maxTime)
{
}

//////////////////////////////////////////////////
Timer::~Timer()
{
}

//////////////////////////////////////////////////
void Timer::Start()
{
  if (this->reset)
  {
    this->start = Time::GetWallTime();
    this->reset = false;
  }
  else if (!this->running)
  {
    // Add the time that has elapsed since stopping to the start time.
    this->start += (Time::GetWallTime() - this->stop);
  }

  this->running = true;
}

//////////////////////////////////////////////////
void Timer::Stop()
{
  this->stop = Time::GetWallTime();
  this->running = false;
}

//////////////////////////////////////////////////
void Timer::Reset()
{
  this->running = false;
  this->reset = true;
  this->start = this->stop = Time::GetWallTime();
}

//////////////////////////////////////////////////
bool Timer::GetRunning() const
{
  return this->running;
}

//////////////////////////////////////////////////
Time Timer::GetElapsed() const
{
  Time elapsedTime;
  if (this->running)
  {
    elapsedTime = Time::GetWallTime() - this->start;
  }
  else
  {
    elapsedTime = this->stop - this->start;
  }

  // If we're counting down, return the countdown time minus the total
  // elapsed time.
  if (this->countdown)
  {
    if (elapsedTime > this->maxTime)
    {
      // If elapsed time is past the countdown time, return 0 (out of time)
      return Time::Zero;
    }
    return this->maxTime - elapsedTime;
  }
  return elapsedTime;
}
