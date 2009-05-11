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
#include <FL/Fl_Gl_Window.H>

#include "Global.hh"
#include "XMLConfig.hh"
#include "GLFrameManager.hh"
#include "OgreAdaptor.hh"
#include "OgreCreator.hh"
#include "Simulator.hh"
#include "GLWindow.hh"
#include "MainMenu.hh"
#include "Toolbar.hh"
#include "StatusBar.hh"
#include "Gui.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
Gui::Gui (int x, int y, int width, int height, const std::string &t)
  : Fl_Window(x, y, width, height, t.c_str())
{
  Fl::scheme("plastic");

  Param::Begin(&this->parameters);
  this->sizeP = new ParamT<Vector2<int> >("size", Vector2<int>(800, 600), 0);
  this->posP = new ParamT<Vector2<int> >("pos",Vector2<int>(0,0),0);
  Param::End();

  // The order of creation matters! Menubar first, followed by FrameManager,
  // then statusbar
  {
    int toolbarWidth = 250;

    // Create a main menu
    new MainMenu(0,0,w(),30,(char *)"MainMenu");

    // Create the frame mamanger
    this->frameMgr = new GLFrameManager(0, 30, 
                         this->w()-toolbarWidth, this->h()-60, "");

    this->toolbar = new Toolbar(this->w() - toolbarWidth, 30,
                                toolbarWidth, this->h() - 60);

    // Create the status bar
    this->statusbar = new StatusBar(0, height-30, 
                         width, 30);

    this->statusbar->gui = this;
  }

  // Create the toolbar
  //this->toolbar = new Toolbar(this->w()-200, 30, 200, this->h()-60);

  this->resizable(this->statusbar);
  //this->resizable(this->toolbar);
  this->resizable(this->frameMgr);

  this->end();
  this->show();

  Fl::check();
  Fl::wait(0.3);

  // Create a dummy rendering window. This creates a context, and allows Ogre
  // to initialize properly
  OgreCreator::Instance()->CreateWindow(this, 1, 1);

  this->hasFocus = true;
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
Gui::~Gui()
{
  this->hide();

  delete this->sizeP;
  delete this->posP;

  //delete this->statusbar;
}

////////////////////////////////////////////////////////////////////////////////
/// Load the gui
void Gui::Load( XMLConfigNode *node )
{
  this->sizeP->Load(node);
  this->posP->Load(node);

  this->frameMgr->Load( node->GetChild("frames") );
}

////////////////////////////////////////////////////////////////////////////////
// Save the gui params in xml format
void Gui::Save(std::string &prefix, std::ostream &stream)
{
  std::string p = prefix + "  ";

  stream << prefix <<  "<rendering:gui>\n";
  stream << prefix <<  "  " << *(this->sizeP) << "\n";
  stream << prefix <<  "  " << *(this->posP) << "\n";
  this->frameMgr->Save(p, stream);
  stream << prefix << "</rendering:gui>\n";
}

////////////////////////////////////////////////////////////////////////////////
/// Initalize the gui
void Gui::Init()
{
  this->frameMgr->Init();
}

////////////////////////////////////////////////////////////////////////////////
void Gui::Update()
{
  this->toolbar->Update();
  this->statusbar->Update();
  this->frameMgr->Update();

  Fl::check();

  //Fl::wait(0.3);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the width of the gui's rendering window
unsigned int Gui::GetWidth() const
{
  return this->w();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the height of the gui's rendering window
unsigned int Gui::GetHeight() const
{
  return this->h();
}

////////////////////////////////////////////////////////////////////////////////
/// Handle an event
int Gui::handle(int event)
{
  switch(event)
  {
    case FL_FOCUS:
      this->hasFocus = true;
      break;
    case FL_UNFOCUS:
      this->hasFocus = false;
      break;
    case FL_HIDE:
      if (this->hasFocus)
        Simulator::Instance()->SetUserQuit();
      break;
  }

  return Fl_Window::handle(event);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the average FPS
float Gui::GetAvgFPS() const
{
  return this->frameMgr->GetFPS();
}


