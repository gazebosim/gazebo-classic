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

#include "gazebo/common/Events.hh"
#include "gazebo/common/Diagnostics.hh"

using namespace gazebo;
using namespace common;

DiagnosticManager *DiagnosticTimer::diagManager = DiagnosticManager::Instance();

//////////////////////////////////////////////////
DiagnosticManager::DiagnosticManager()
{
}

//////////////////////////////////////////////////
DiagnosticManager::~DiagnosticManager()
{
}

//////////////////////////////////////////////////
DiagnosticTimerPtr DiagnosticManager::CreateTimer(const std::string &_name)
{
  return DiagnosticTimerPtr(new DiagnosticTimer(_name));
}

//////////////////////////////////////////////////
void DiagnosticManager::TimerStart(DiagnosticTimer *_timer)
{
  this->timers[_timer->GetName()] = Time();
  event::Events::diagTimerStart(_timer->GetName());
}

//////////////////////////////////////////////////
void DiagnosticManager::TimerStop(DiagnosticTimer *_timer)
{
  this->timers[_timer->GetName()] = _timer->GetElapsed();
  event::Events::diagTimerStop(_timer->GetName());
}

//////////////////////////////////////////////////
int DiagnosticManager::GetTimerCount() const
{
  return this->timers.size();
}

//////////////////////////////////////////////////
Time DiagnosticManager::GetTime(int _index) const
{
  std::map<std::string, Time>::const_iterator iter;

  iter = this->timers.begin();
  std::advance(iter, _index);

  if (iter != this->timers.end())
    return iter->second;
  else
    gzerr << "Error getting time\n";

  return Time();
}

//////////////////////////////////////////////////
std::string DiagnosticManager::GetLabel(int _index) const
{
  std::map<std::string, Time>::const_iterator iter;

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
  std::map<std::string, Time>::const_iterator iter;
  iter = this->timers.find(_label);

  if (iter != this->timers.end())
    return iter->second;
  else
    gzerr << "Error getting time\n";

  return Time();
}
