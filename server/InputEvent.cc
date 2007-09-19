/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: Gazebo input event
 * Author: Nate Koenig
 * Date: 17 Sep 2007
 * SVN: $Id:$
 */

#include "InputEvent.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
InputEvent::InputEvent()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
InputEvent::~InputEvent()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Set the type of event
void InputEvent::SetType(Type t)
{
  this->type = t;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the type of event
InputEvent::Type InputEvent::GetType() const
{
  return this->type;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the key pressed or released
void InputEvent::SetKey(int k)
{
  this->key = k;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the key pressed of released
int InputEvent::GetKey() const
{
  return this->key;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the mouse position
void InputEvent::SetMousePos(const Vector2<int> pos)
{
  this->mousePos = pos;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the mouse position
Vector2<int> InputEvent::GetMousePos() const
{
  return this->mousePos;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the mouse button that was pressed/released
void InputEvent::SetMouseButton(InputEvent::MouseButton button)
{
  this->mouseButton = button;
}
 
////////////////////////////////////////////////////////////////////////////////
/// Get the mouse button that was pressed/released
InputEvent::MouseButton InputEvent::GetMouseButton() const
{
  return this->mouseButton;
}

