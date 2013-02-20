/*
 * Copyright 2012 Open Source Robotics Foundation
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
/* Desc: A diagnostic class
 * Author: Nate Koenig
 * Date: 2 Feb 2011
 */

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Diagnostics.hh"

using namespace gazebo;
using namespace common;

//////////////////////////////////////////////////
DiagnosticManager::DiagnosticManager()
{
  // Get the base of the time logging path
  if (!getenv("HOME"))
  {
    gzwarn << "HOME environment variable missing. Diagnostic timing "
      << "information will be logged to /tmp/gazebo.\n";
    this->logPath = "/tmp/gazebo";
  }
  else
  {
    this->logPath = getenv("HOME");
    this->logPath /= ".gazebo";
  }

  this->logPath = this->logPath / "timing" /
    common::Time::GetWallTimeAsISOString();

  // Make sure the path exists.
  if (!boost::filesystem::exists(this->logPath))
    boost::filesystem::create_directories(this->logPath);
}

//////////////////////////////////////////////////
DiagnosticManager::~DiagnosticManager()
{
}

//////////////////////////////////////////////////
boost::filesystem::path DiagnosticManager::GetLogPath() const
{
  return this->logPath;
}

//////////////////////////////////////////////////
void DiagnosticManager::StartTimer(const std::string &_name)
{
  TimerMap::iterator iter = this->timers.find(_name);
  if (iter != this->timers.end())
  {
    GZ_ASSERT(iter->second != NULL, "DiagnosticTimerPtr is NULL");
    iter->second->Start();
  }
  else
  {
    this->timers[_name] = DiagnosticTimerPtr(new DiagnosticTimer(_name));
  }
}

//////////////////////////////////////////////////
void DiagnosticManager::StopTimer(const std::string &_name)
{
  TimerMap::iterator iter = this->timers.find(_name);
  if (iter != this->timers.end())
  {
    GZ_ASSERT(iter->second, "DiagnosticTimerPtr is NULL");
    iter->second->Stop();
  }
  else
    gzerr << "Unable to find timer[" << _name << "]\n";
}

//////////////////////////////////////////////////
void DiagnosticManager::Lap(const std::string &_name,
                            const std::string &_prefix)
{
  TimerMap::iterator iter;
  iter = this->timers.find(_name);

  if (iter == this->timers.end())
    gzerr << "Unable to find timer with name[" << _name << "]\n";
  else
  {
    GZ_ASSERT(iter->second, "DiagnosticTimerPtr is NULL");

    iter->second->Lap(_prefix);
  }
}

//////////////////////////////////////////////////
int DiagnosticManager::GetTimerCount() const
{
  return this->timers.size();
}

//////////////////////////////////////////////////
Time DiagnosticManager::GetTime(int _index) const
{
  TimerMap::const_iterator iter;

  iter = this->timers.begin();
  std::advance(iter, _index);

  if (iter != this->timers.end())
  {
    GZ_ASSERT(iter->second, "DiagnosticTimerPtr is NULL");
    return iter->second->GetElapsed();
  }
  else
    gzerr << "Error getting time\n";

  return Time();
}

//////////////////////////////////////////////////
std::string DiagnosticManager::GetLabel(int _index) const
{
  TimerMap::const_iterator iter;

  iter = this->timers.begin();
  std::advance(iter, _index);

  if (iter != this->timers.end())
    return iter->first;
  else
    gzerr << "Erorr getting label\n";

  return "null";
}

//////////////////////////////////////////////////
Time DiagnosticManager::GetTime(const std::string &_label) const
{
  TimerMap::const_iterator iter;
  iter = this->timers.find(_label);

  if (iter != this->timers.end())
  {
    GZ_ASSERT(iter->second, "DiagnosticTimerPtr is NULL");
    return iter->second->GetElapsed();
  }
  else
    gzerr << "Error getting time\n";

  return Time();
}

//////////////////////////////////////////////////
DiagnosticTimer::DiagnosticTimer(const std::string &_name) : Timer()
{
  boost::filesystem::path logPath;

  logPath = DiagnosticManager::Instance()->GetLogPath() / (_name + ".log");
  this->log.open(logPath.string().c_str(), std::ios::out | std::ios::app);

  this->name = _name;
  this->Start();
}

//////////////////////////////////////////////////
DiagnosticTimer::~DiagnosticTimer()
{
  this->Stop();
  this->log.close();
}

//////////////////////////////////////////////////
void DiagnosticTimer::Start()
{
  // Only start if not running.
  if (!this->GetRunning())
  {
    // Start the timer
    Timer::Start();

    // Make sure the prev lap is reset
    this->prevLap.Set(0, 0);
  }
}

//////////////////////////////////////////////////
void DiagnosticTimer::Stop()
{
  // Only stop is currently running
  if (this->GetRunning())
  {
    // Stop the timer
    Timer::Stop();

    // Write out the total elapsed time.
    this->log << this->name << " " << this->GetElapsed().Double() << std::endl;
    this->log.flush();

    // Reset the lap time
    this->prevLap.Set(0, 0);
  }
}

//////////////////////////////////////////////////
void DiagnosticTimer::Lap(const std::string &_prefix)
{
  // Get the current elapsed time.
  common::Time currTime = this->GetElapsed();

  // Write out the delta time.
  this->log << this->name << ":" << _prefix << " "
    << (currTime - this->prevLap).Double() << std::endl;

  // Store the prev lap time.
  this->prevLap = currTime;
}
