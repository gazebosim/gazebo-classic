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

#ifndef INPUTEVENT_HH
#define INPUTEVENT_HH

#include "Vector2.hh"

namespace gazebo
{

  /// \brief InputEvent Class
  class InputEvent
  {
    /// Types of events
    public: enum Type {KEY_PRESS, 
                       KEY_RELEASE, 
                       MOUSE_PRESS, 
                       MOUSE_RELEASE,
                       MOUSE_DRAG};

    /// Mouse buttons
    public: enum MouseButton {NONE, LEFT_MOUSE, RIGHT_MOUSE, MIDDLE_MOUSE};


    /// \brief Constructor
    public: InputEvent ();

    /// \brief Destructor
    public: virtual ~InputEvent();

    /// \brief Set the type of event
    public: void SetType(Type t);

    /// \brief Get the type of event
    public: Type GetType() const;

    /// \brief Set the key pressed or released
    public: void SetKey(int k);

    /// \brief Get the key pressed of released
    public: int GetKey() const;

    /// \brief Set the mouse position
    public: void SetMousePos(const Vector2<int> pos);

    /// \brief Get the mouse position
    public: Vector2<int> GetMousePos() const;

    /// \brief Set the mouse button that was pressed/released
    public: void SetMouseButton(MouseButton button);

    /// \brief Get the mouse button that was pressed/released
    public: MouseButton GetMouseButton() const;

    private: Type type;
    private: int key;
    private: Vector2<int> mousePos;
    private: MouseButton mouseButton;

  };
}


#endif /* INPUTEVENT_HH */
