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
/* Desc: GUI base class
 * Author: Nate Koenig
 * Date: 17 Sep 2007
 * SVN: $Id$
 */

#include "OgreAdaptor.hh"
#include "Gui.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
///Constructor
Gui::Gui()
{
  this->windowId = -1;
  this->visual = NULL;
  this->display = NULL;

  this->ogre = OgreAdaptor::Instance();
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
Gui::~Gui()
{
}
////////////////////////////////////////////////////////////////////////////////
/// Get the id of the window
Window Gui::GetWindowId() const
{
  return this->windowId;
}       

////////////////////////////////////////////////////////////////////////////////
/// Get the visual info
XVisualInfo *Gui::GetVisualInfo() const
{
  return this->visual;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the display
Display *Gui::GetDisplay() const
{
  return this->display;
}

////////////////////////////////////////////////////////////////////////////////
/// Handle a keyboard event
void Gui::HandleKeyboard(int key)
{
}
