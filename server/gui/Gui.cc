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
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 * SVN: $Id$
 */

#include <string>

#include <FL/Fl_Menu_Item.H>
#include <FL/Fl_Menu_Bar.H>
#include <FL/Fl_Choice.H>

#include "GLWindow.hh"
#include "Global.hh"
#include "Simulator.hh"
#include "MainMenu.hh"
#include "Toolbar.hh"
#include "StatusBar.hh"
#include "Gui.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
Gui::Gui (int x, int y, int width, int height, const std::string &t)
  : Fl_Window(x, y, width+200, height+60, t.c_str())
{
  this->windowId = -1;
  this->visual = NULL;
  this->display = NULL;

  Fl::scheme("plastic");

  // Create a main menu
  MainMenu *mainMenu = new MainMenu(0,0,w(),30,(char *)"MainMenu");

  // Create the Rendering window
  this->glWindow = new GLWindow(0, 30, w()-200, h()-60,"GL Window");

  // Create the toolbar
  this->toolbar = new Toolbar(w()-200, 30, 200, h()-60);

  this->statusbar = new StatusBar(0, h()-30, w(), 30);

  this->end();
  this->show();

  this->glWindow->Init();

  this->display = this->glWindow->display;
  this->visual = this->glWindow->visual;
  this->colormap = this->glWindow->colormap;
  this->windowId = this->glWindow->windowId;  
  this->resizable(this->glWindow);
  //this->glWindow->UserQuit.connect( &gazebo::Gui::UserQuit);
  //this->glWindow->Finished.connect( boost::bind(&gazebo::Gui::Finished,this));
  //MainMenu::Finished.connect( boost::bind(&gazebo::Gui::Finished,this));
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
Gui::~Gui()
{
  delete this->glWindow;
  delete this->toolbar;
  //delete this->statusbar;
  
}

////////////////////////////////////////////////////////////////////////////////
/// Initalize the gui
void Gui::Init()
{
}

////////////////////////////////////////////////////////////////////////////////
void Gui::Update()
{
  this->toolbar->Update();
  this->statusbar->Update();
  this->glWindow->Update();
  Fl::wait(0.03);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the width of the gui's rendering window
unsigned int Gui::GetWidth() const
{
  return this->glWindow->w();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the height of the gui's rendering window
unsigned int Gui::GetHeight() const
{
  return this->glWindow->h();
}

////////////////////////////////////////////////////////////////////////////////
/// Handle an event
int Gui::handle(int event)
{
  switch (event)
  {
    case FL_HIDE:
      Simulator::Instance()->SetUserQuit();
      return 1;
  }

  return Fl_Window::handle(event);
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
