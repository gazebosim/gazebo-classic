/*
 * Copyright 2012 Open Source Robotics Foundation
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
  this->pressFilters.push_front(_filter);
}

/////////////////////////////////////////////////
void MouseEventHandler::AddReleaseFilter(const std::string &_name,
    MouseEventFilter _filter)
{
  this->releaseFilters.push_front(_filter);
}

/////////////////////////////////////////////////
void MouseEventHandler::AddMoveFilter(const std::string &_name,
    MouseEventFilter _filter)
{
  this->moveFilters.push_front(_filter);
}

/////////////////////////////////////////////////
void MouseEventHandler::HandlePress(const common::MouseEvent &_event)
{
  this->Handle(this->pressFilters, _event);
}

/////////////////////////////////////////////////
void MouseEventHandler::HandleRelease(const common::MouseEvent &_event)
{
  this->Handle(this->releaseFilters, _event);
}

/////////////////////////////////////////////////
void MouseEventHandler::HandleMove(const common::MouseEvent &_event)
{
  this->Handle(this->moveFilters, _event);
}

/////////////////////////////////////////////////
void MouseEventHandler::Handle(std::list<MouseEventFilter> &_list,
  const common::MouseEvent &_event)
{
  for (std::list<MouseEventFilter>::iterator iter = _list.begin();
       iter != _list.end(); ++iter)
  {
    if ((*iter)(_event))
      break;
  }
}
