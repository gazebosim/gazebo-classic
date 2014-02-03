/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include "gazebo/gui/KeyEventHandler.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
KeyEventHandler::KeyEventHandler()
{
}

/////////////////////////////////////////////////
KeyEventHandler::~KeyEventHandler()
{
}

/////////////////////////////////////////////////
void KeyEventHandler::AddPressFilter(const std::string &_name,
    KeyEventFilter _filter)
{
  this->Add(_name, _filter, this->pressFilters);
}

/////////////////////////////////////////////////
void KeyEventHandler::AddReleaseFilter(const std::string &_name,
    KeyEventFilter _filter)
{
  this->Add(_name, _filter, this->releaseFilters);
}

/////////////////////////////////////////////////
void KeyEventHandler::RemovePressFilter(const std::string &_name)
{
  this->Remove(_name, this->pressFilters);
}

/////////////////////////////////////////////////
void KeyEventHandler::RemoveReleaseFilter(const std::string &_name)
{
  this->Remove(_name, this->releaseFilters);
}

/////////////////////////////////////////////////
void KeyEventHandler::HandlePress(const common::KeyEvent &_event)
{
  this->Handle(_event, this->pressFilters);
}

/////////////////////////////////////////////////
void KeyEventHandler::HandleRelease(const common::KeyEvent &_event)
{
  this->Handle(_event, this->releaseFilters);
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
void KeyEventHandler::Handle(const common::KeyEvent &_event,
    std::list<Filter> &_list)
{
  for (std::list<Filter>::iterator iter = _list.begin();
       iter != _list.end(); ++iter)
  {
    if ((*iter).func(_event))
      break;
  }
}
