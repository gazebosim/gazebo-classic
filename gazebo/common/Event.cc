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
EventPrivate::EventPrivate()
  : signaled(false)
{
}

//////////////////////////////////////////////////
Event::Event()
  : dataPtr(new EventPrivate())
{
}

//////////////////////////////////////////////////
Event::Event(EventPrivate &_d)
  : dataPtr(&_d)
{
}

//////////////////////////////////////////////////
Event::~Event()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

//////////////////////////////////////////////////
bool Event::GetSignaled() const
{
  return this->dataPtr->signaled;
}

//////////////////////////////////////////////////
ConnectionPrivate::ConnectionPrivate()
  : event(NULL), id(-1)
{
}

//////////////////////////////////////////////////
ConnectionPrivate::ConnectionPrivate(Event *_e, int _i)
  : event(_e), id(_i)
{
}

//////////////////////////////////////////////////
Connection::Connection()
  : dataPtr(new ConnectionPrivate())
{
}

//////////////////////////////////////////////////
Connection::Connection(Event *_e, int _i)
  : dataPtr(new ConnectionPrivate(_e, _i))
{
  this->dataPtr->creationTime = common::Time::GetWallTime();
}

//////////////////////////////////////////////////
Connection::~Connection()
{
gzdbg << "A  " << this << std::endl;
  common::Time diffTime = common::Time::GetWallTime() -
    this->dataPtr->creationTime;
gzdbg << "A  " << this << " ---  " << diffTime << "   " << this->dataPtr->event  << std::endl;
gzdbg << "A  " << this << " ---  " << (diffTime < common::Time(0, 10000)) << std::endl;
gzdbg << "A  " << this << " ---  " << diffTime << "   " << this->dataPtr->event  << std::endl;
//  if ((this->dataPtr->event && !this->dataPtr->event->GetSignaled()) &&
//      diffTime < common::Time(0, 10000))
//  {
  //  gzwarn << "Warning: Deleteing a connection right after creation. "
    //      << "Make sure to save the ConnectionPtr from a Connect call\n";
//  }
gzdbg << "A  " << this << std::endl;

  if (this->dataPtr->event && this->dataPtr->id >= 0)
  {
gzdbg << "A  " << this << std::endl;
    this->dataPtr->event->Disconnect(this->dataPtr->id);
gzdbg << "A  " << this << std::endl;
    this->dataPtr->id = -1;
gzdbg << "A  " << this << std::endl;
    this->dataPtr->event = NULL;
  }
gzdbg << "B  " << this << std::endl;
}

//////////////////////////////////////////////////
int Connection::GetId() const
{
  return this->dataPtr->id;
}
