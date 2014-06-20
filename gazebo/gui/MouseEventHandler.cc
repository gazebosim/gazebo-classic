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

#include "gazebo/gui/MouseEventHandler.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
MouseEventHandler::MouseEventHandler()
{
}

/////////////////////////////////////////////////
MouseEventHandler::~MouseEventHandler()
{
}

/////////////////////////////////////////////////
void MouseEventHandler::AddPressFilter(const std::string &_name,
    MouseEventFilter _filter)
{
  this->Add(_name, _filter, this->pressFilters);
}

/////////////////////////////////////////////////
void MouseEventHandler::AddReleaseFilter(const std::string &_name,
    MouseEventFilter _filter)
{
  this->Add(_name, _filter, this->releaseFilters);
}

/////////////////////////////////////////////////
void MouseEventHandler::AddMoveFilter(const std::string &_name,
    MouseEventFilter _filter)
{
  this->Add(_name, _filter, this->moveFilters);
}

/////////////////////////////////////////////////
void MouseEventHandler::RemovePressFilter(const std::string &_name)
{
  this->Remove(_name, this->pressFilters);
}

/////////////////////////////////////////////////
void MouseEventHandler::RemoveReleaseFilter(const std::string &_name)
{
  this->Remove(_name, this->releaseFilters);
}

/////////////////////////////////////////////////
void MouseEventHandler::RemoveMoveFilter(const std::string &_name)
{
  this->Remove(_name, this->moveFilters);
}

/////////////////////////////////////////////////
void MouseEventHandler::HandlePress(const common::MouseEvent &_event)
{
  this->Handle(_event, this->pressFilters);
}

/////////////////////////////////////////////////
void MouseEventHandler::HandleRelease(const common::MouseEvent &_event)
{
  this->Handle(_event, this->releaseFilters);
}

/////////////////////////////////////////////////
void MouseEventHandler::HandleMove(const common::MouseEvent &_event)
{
  this->Handle(_event, this->moveFilters);
}

/////////////////////////////////////////////////
void MouseEventHandler::Add(const std::string &_name,
    MouseEventFilter _filter, std::list<Filter> &_list)
{
  std::list<Filter>::iterator iter =
    std::find(_list.begin(), _list.end(), _name);

  if (iter == _list.end())
    _list.push_front(Filter(_name, _filter));
}

/////////////////////////////////////////////////
void MouseEventHandler::Remove(const std::string &_name,
    std::list<Filter> &_list)
{
  _list.remove(Filter(_name, NULL));
}

/////////////////////////////////////////////////
void MouseEventHandler::Handle(const common::MouseEvent &_event,
    std::list<Filter> &_list)
{
  for (std::list<Filter>::iterator iter = _list.begin();
       iter != _list.end(); ++iter)
  {
    if ((*iter).func(_event))
      break;
  }
}
