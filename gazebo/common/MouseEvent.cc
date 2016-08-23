/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License")
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

#include "gazebo/common/MouseEventPrivate.hh"
#include "gazebo/common/MouseEvent.hh"

using namespace gazebo;
using namespace common;

/////////////////////////////////////////////////
MouseEvent::MouseEvent()
  : dataPtr(new MouseEventPrivate)
{
}

/////////////////////////////////////////////////
MouseEvent::MouseEvent(const MouseEvent &_other)
  : dataPtr(new MouseEventPrivate)
{
  *dataPtr = *_other.dataPtr;
}

/////////////////////////////////////////////////
MouseEvent::~MouseEvent()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
ignition::math::Vector2i MouseEvent::Pos() const
{
  return this->dataPtr->pos;
}

/////////////////////////////////////////////////
void MouseEvent::SetPos(const ignition::math::Vector2i &_pos)
{
  this->dataPtr->pos = _pos;
}

/////////////////////////////////////////////////
void MouseEvent::SetPos(const int _x, const int _y)
{
  this->dataPtr->pos.Set(_x, _y);
}

/////////////////////////////////////////////////
ignition::math::Vector2i MouseEvent::PrevPos() const
{
  return this->dataPtr->prevPos;
}

/////////////////////////////////////////////////
void MouseEvent::SetPrevPos(const ignition::math::Vector2i &_pos)
{
  this->dataPtr->prevPos = _pos;
}

/////////////////////////////////////////////////
void MouseEvent::SetPrevPos(const int _x, const int _y)
{
  this->dataPtr->prevPos.Set(_x, _y);
}

/////////////////////////////////////////////////
ignition::math::Vector2i MouseEvent::PressPos() const
{
  return this->dataPtr->pressPos;
}

/////////////////////////////////////////////////
void MouseEvent::SetPressPos(const ignition::math::Vector2i &_pos)
{
  this->dataPtr->pressPos = _pos;
}

/////////////////////////////////////////////////
void MouseEvent::SetPressPos(const int _x, const int _y)
{
  this->dataPtr->pressPos.Set(_x, _y);
}

/////////////////////////////////////////////////
ignition::math::Vector2i MouseEvent::Scroll() const
{
  return this->dataPtr->scroll;
}

/////////////////////////////////////////////////
void MouseEvent::SetScroll(const ignition::math::Vector2i &_scroll)
{
  this->dataPtr->scroll = _scroll;
}

/////////////////////////////////////////////////
void MouseEvent::SetScroll(const int _x, const int _y)
{
  this->dataPtr->scroll.Set(_x, _y);
}

/////////////////////////////////////////////////
float MouseEvent::MoveScale() const
{
  return this->dataPtr->moveScale;
}

/////////////////////////////////////////////////
void MouseEvent::SetMoveScale(const float _scale)
{
  this->dataPtr->moveScale = _scale;
}

/////////////////////////////////////////////////
bool MouseEvent::Dragging() const
{
  return this->dataPtr->dragging;
}

/////////////////////////////////////////////////
void MouseEvent::SetDragging(const bool _dragging)
{
  this->dataPtr->dragging = _dragging;
}

/////////////////////////////////////////////////
MouseEvent::EventType MouseEvent::Type() const
{
  return this->dataPtr->type;
}

/////////////////////////////////////////////////
void MouseEvent::SetType(const EventType _type) const
{
  this->dataPtr->type = _type;
}

/////////////////////////////////////////////////
MouseEvent::MouseButton MouseEvent::Button() const
{
  return this->dataPtr->button;
}

/////////////////////////////////////////////////
void MouseEvent::SetButton(const MouseEvent::MouseButton _button) const
{
  this->dataPtr->button = _button;
}

/////////////////////////////////////////////////
unsigned int MouseEvent::Buttons() const
{
  return this->dataPtr->buttons;
}

/////////////////////////////////////////////////
void MouseEvent::SetButtons(const unsigned int &_buttons)
{
  this->dataPtr->buttons = _buttons;
}

/////////////////////////////////////////////////
bool MouseEvent::Shift() const
{
  return this->dataPtr->shift;
}

/////////////////////////////////////////////////
void MouseEvent::SetShift(const bool _shift) const
{
  this->dataPtr->shift = _shift;
}

/////////////////////////////////////////////////
bool MouseEvent::Alt() const
{
  return this->dataPtr->alt;
}

/////////////////////////////////////////////////
void MouseEvent::SetAlt(const bool _alt)
{
  this->dataPtr->alt = _alt;
}

/////////////////////////////////////////////////
bool MouseEvent::Control() const
{
  return this->dataPtr->control;
}

/////////////////////////////////////////////////
void MouseEvent::SetControl(const bool _control) const
{
  this->dataPtr->control = _control;
}

/////////////////////////////////////////////////
MouseEvent &MouseEvent::operator=(const MouseEvent &_other)
{
  if (this == &_other)
      return *this;

  *this->dataPtr = *_other.dataPtr;

  return *this;
}
