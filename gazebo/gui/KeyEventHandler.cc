/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include <list>
#include <string>
#include "gazebo/common/KeyEvent.hh"
#include "gazebo/gui/KeyEventHandler.hh"
#include "gazebo/gui/KeyEventHandlerPrivate.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
KeyEventHandler::KeyEventHandler()
  : dataPtr(new KeyEventHandlerPrivate())
{
  this->dataPtr->autoRepeat = false;
}

/////////////////////////////////////////////////
KeyEventHandler::~KeyEventHandler()
{
}

/////////////////////////////////////////////////
void KeyEventHandler::AddPressFilter(const std::string &_name,
    KeyEventFilter _filter)
{
  this->Add(_name, _filter, this->dataPtr->pressFilters);
}

/////////////////////////////////////////////////
void KeyEventHandler::AddReleaseFilter(const std::string &_name,
    KeyEventFilter _filter)
{
  this->Add(_name, _filter, this->dataPtr->releaseFilters);
}

/////////////////////////////////////////////////
void KeyEventHandler::RemovePressFilter(const std::string &_name)
{
  this->Remove(_name, this->dataPtr->pressFilters);
}

/////////////////////////////////////////////////
void KeyEventHandler::RemoveReleaseFilter(const std::string &_name)
{
  this->Remove(_name, this->dataPtr->releaseFilters);
}

/////////////////////////////////////////////////
bool KeyEventHandler::HandlePress(const common::KeyEvent &_event)
{
  return this->Handle(_event, this->dataPtr->pressFilters);
}

/////////////////////////////////////////////////
bool KeyEventHandler::HandleRelease(const common::KeyEvent &_event)
{
  return this->Handle(_event, this->dataPtr->releaseFilters);
}

/////////////////////////////////////////////////
bool KeyEventHandler::AutoRepeat() const
{
  return this->dataPtr->autoRepeat;
}

/////////////////////////////////////////////////
void KeyEventHandler::SetAutoRepeat(const bool _autorepeat)
{
  this->dataPtr->autoRepeat = _autorepeat;
}

/////////////////////////////////////////////////
void KeyEventHandler::Add(const std::string &_name,
    KeyEventFilter _filter, std::list<Filter> &_list)
{
  std::list<Filter>::iterator iter =
    std::find(_list.begin(), _list.end(), _name);

  if (iter == _list.end())
    _list.push_front(Filter(_name, _filter));
}

/////////////////////////////////////////////////
void KeyEventHandler::Remove(const std::string &_name,
    std::list<Filter> &_list)
{
  _list.remove(Filter(_name, NULL));
}

/////////////////////////////////////////////////
bool KeyEventHandler::Handle(const common::KeyEvent &_event,
    std::list<Filter> &_list)
{
  for (std::list<Filter>::iterator iter = _list.begin();
       iter != _list.end(); ++iter)
  {
    if ((*iter).func(_event))
      return true;
  }
  return false;
}
