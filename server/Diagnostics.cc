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

#include "Events.hh"
#include "Diagnostics.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
DiagnosticManager::DiagnosticManager()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
DiagnosticManager::~DiagnosticManager()
{
}


////////////////////////////////////////////////////////////////////////////////
/// A diagnostic timer has started
void DiagnosticManager::TimerStart(DiagnosticTimer *timer)
{
  this->timers[timer->GetName()] = Time();
  Events::diagTimerStartSignal(timer->GetName());
}


////////////////////////////////////////////////////////////////////////////////
/// A diagnostic timer has stoped
void DiagnosticManager::TimerStop(DiagnosticTimer *timer)
{
  this->timers[timer->GetName()] = timer->GetElapsed();
  Events::diagTimerStopSignal(timer->GetName());
}

////////////////////////////////////////////////////////////////////////////////
/// Get the number of timers
int DiagnosticManager::GetTimerCount() const
{
  return this->timers.size();
}

////////////////////////////////////////////////////////////////////////////////
/// Get a specific time
Time DiagnosticManager::GetTime(int index) const
{
  std::map<std::string, Time>::const_iterator iter;

  iter = this->timers.begin();
  std::advance(iter,index);

  if (iter != this->timers.end())
    return iter->second;
  else
    std::cerr << "Error\n";

  return Time();
}

////////////////////////////////////////////////////////////////////////////////
/// Get a label for a timer
std::string DiagnosticManager::GetLabel(int index) const
{
  std::map<std::string, Time>::const_iterator iter;

  iter = this->timers.begin();
  std::advance(iter,index);

  if (iter != this->timers.end())
    return iter->first;
  else
    std::cerr << "Error\n";

  return "null";
}

////////////////////////////////////////////////////////////////////////////////
/// Get a time based on a label
Time DiagnosticManager::GetTime(const std::string &label) const
{
  std::map<std::string, Time>::const_iterator iter;
  iter = this->timers.find(label);

  if (iter != this->timers.end())
    return iter->second;
  else
    std::cerr << "More errors\n";

  return Time();
}


