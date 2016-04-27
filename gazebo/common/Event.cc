/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include "gazebo/common/Console.hh"
#include "gazebo/common/Event.hh"

using namespace gazebo;
using namespace event;

//////////////////////////////////////////////////
Event::Event()
  : signaled(false)
{
}

//////////////////////////////////////////////////
Event::~Event()
{
}

//////////////////////////////////////////////////
bool Event::GetSignaled() const
{
  return this->Signaled();
}

//////////////////////////////////////////////////
bool Event::Signaled() const
{
  return this->signaled;
}

//////////////////////////////////////////////////
void Event::SetSignaled(const bool _sig)
{
  this->signaled = _sig;
}

//////////////////////////////////////////////////
Connection::Connection(Event *_e, const int _i)
  : event(_e), id(_i)
{
  this->creationTime = common::Time::GetWallTime();
}

//////////////////////////////////////////////////
Connection::~Connection()
{
  common::Time diffTime = common::Time::GetWallTime() - this->creationTime;
  if ((this->event && !this->event->Signaled()) &&
      diffTime < common::Time(0, 10000))
  {
    gzwarn << "Warning: Deleting a connection right after creation. "
          << "Make sure to save the ConnectionPtr from a Connect call\n";
  }

  if (this->event && this->id >= 0)
  {
    this->event->Disconnect(this->id);
    this->id = -1;
    this->event = nullptr;
  }
}

//////////////////////////////////////////////////
int Connection::GetId() const
{
  return this->Id();
}

//////////////////////////////////////////////////
int Connection::Id() const
{
  return this->id;
}
