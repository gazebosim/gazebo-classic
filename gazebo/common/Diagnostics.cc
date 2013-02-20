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

#include <boost/filesystem.hpp>
#include <boost/smart_ptr.hpp>

#include "gazebo/common/Events.hh"
#include "gazebo/common/Diagnostics.hh"

using namespace gazebo;
using namespace common;

// DiagnosticManager *DiagnosticTimer::diagManager = DiagnosticManager::Instance();

//////////////////////////////////////////////////
DiagnosticManager::DiagnosticManager()
{
  boost::filesystem::path logPath;

  // Get the base of the time logging path
  if (!getenv("HOME"))
  {
    gzwarn << "HOME environment variable missing. Diagnostic timing "
      << "information will be logged to /tmp/gazebo.\n";
    logPath = "/tmp/gazebo";
  }
  else
  {
    logPath = getenv("HOME");
    logPath /= ".gazebo";
  }

  logPath /= "timing";

  // Make sure the path exists.
  if (!boost::filesystem::exists(logPath))
    boost::filesystem::create_directory(logPath);
}

//////////////////////////////////////////////////
DiagnosticManager::~DiagnosticManager()
{
}

//////////////////////////////////////////////////
DiagnosticTimerPtr DiagnosticManager::CreateTimer(const std::string &_name)
{
  DiagnosticTimerPtr result(new DiagnosticTimer(_name));
  this->timers.insert(
      std::pair<std::string, DiagnostTimerWeakPtr>(_name, result));
  return result;
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
    if (DiagnosticTimerPtr ptr = iter->second.lock())
    {

      std::cerr << "Lap[" << _name << ":" << _prefix << "] " << 
        ptr->GetElapsed().Double(); 
      ptr->Start();
      // event::Events::diagTimerLap(_name, _prefix);
    }
  }
}

//////////////////////////////////////////////////
/*void DiagnosticManager::TimerStart(DiagnosticTimer *_timer)
{
  this->timers[_timer->GetName()] = Timer();
}

//////////////////////////////////////////////////
void DiagnosticManager::TimerStop(DiagnosticTimer *_timer)
{
  this->timers[_timer->GetName()] = _timer->GetElapsed();
  event::Events::diagTimerStop(_timer->GetName());
}*/

//////////////////////////////////////////////////
int DiagnosticManager::GetTimerCount() const
{
  return this->timers.size();
}

//////////////////////////////////////////////////
Time DiagnosticManager::GetTime(int _index) const
{
  DiagnosticTimerPtr timer;
  TimerMap::const_iterator iter;

  iter = this->timers.begin();
  std::advance(iter, _index);

  if (iter != this->timers.end() && (timer = iter->second.lock()))
    return timer->GetElapsed();
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
  DiagnosticTimerPtr timer;

  TimerMap::const_iterator iter;
  iter = this->timers.find(_label);

  if (iter != this->timers.end() && (timer = iter->second.lock()))
    return timer->GetElapsed();
  else
    gzerr << "Error getting time\n";

  return Time();
}
