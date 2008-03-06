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
/* Desc: FLTK Mainwindow
 * Author: Renaud Barate
 * Date: 16 Feb 2008
 */

#include "Global.hh"
#include "DummyGui.hh"
#include "GuiFactory.hh"
#include <time.h>

using namespace gazebo;

GZ_REGISTER_STATIC_GUI("dummy", DummyGui);

////////////////////////////////////////////////////////////////////////////////
/// Constructor: Creates a dummy window necessary to initialize and use GL rendering system.
/// This window won't appear on the screen and is not used afterward.
DummyGui::DummyGui(int x, int y, int width, int height, const std::string &t): 
Gui (x,y,width,height, t)
{
  this->display = XOpenDisplay(0);
  if (!this->display) gzthrow(std::string("Can't open display: ") + XDisplayName(0) + "\n");
  int screen = DefaultScreen(this->display);
  int attribList[8] =
      {GLX_RGBA, GLX_RED_SIZE, 8, GLX_GREEN_SIZE, 8,
      GLX_BLUE_SIZE, 8, None};
  this->visual = glXChooseVisual(this->display, screen, (int *)attribList);
  this->windowId = XCreateSimpleWindow(this->display, RootWindow(this->display, screen), 0, 0, 1, 1, 0, 0, 0);
  this->context = glXCreateContext(this->display, this->visual, NULL, 1);
  glXMakeCurrent(this->display, this->windowId, this->context);
}


////////////////////////////////////////////////////////////////////////////////
/// Destructor: Deletes the dummy window and closes the display
DummyGui::~DummyGui()
{
  glXDestroyContext(this->display, this->context);
  XDestroyWindow(this->display, this->windowId);
  XCloseDisplay(this->display);
}

////////////////////////////////////////////////////////////////////////////////
void DummyGui::Update()
{
  timespec sleepTime={0, 1000000};
  nanosleep(&sleepTime,0);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the id of the window
Window DummyGui::GetWindowId() const
{
  return this->windowId;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the visual info
XVisualInfo *DummyGui::GetVisualInfo() const
{
  return this->visual;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the display
Display *DummyGui::GetDisplay() const
{
  return this->display;
}

      
