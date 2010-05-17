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

#include <X11/xpm.h>
#include <FL/Fl_Menu_Item.H>
#include <FL/Fl_Menu_Bar.H>
#include <FL/Fl_Choice.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/fl_draw.H>
#include <FL/x.H>

#include "World.hh"
#include "Global.hh"
#include "XMLConfig.hh"
#include "GLFrameManager.hh"
#include "OgreAdaptor.hh"
#include "OgreCreator.hh"
#include "Simulator.hh"
#include "GLWindow.hh"
#include "MainMenu.hh"
#include "Toolbar.hh"
#include "Sidebar.hh"
#include "StatusBar.hh"
#include "Gui.hh"

#include "icon.xpm"

using namespace gazebo;

double Gui::forceMultiplier = 1;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
Gui::Gui (int x, int y, int width, int height, const std::string &t)
  : Fl_Double_Window(x, y, width, height, t.c_str())
{
  this->color(BG_COLOR);

  Param::Begin(&this->parameters);
  this->sizeP = new ParamT<Vector2<int> >("size", Vector2<int>(800, 600), 0);
  this->posP = new ParamT<Vector2<int> >("pos",Vector2<int>(0,0),0);
  Param::End();

  width = std::max(800, this->w());
  height = std::max(480, this->h());

  this->w(width);
  this->h(height);

  // The order of creation matters! Menubar first, followed by FrameManager,
  // then statusbar
  {
    int toolbarWidth = 250;

    // Create a main menu
    new MainMenu(0, 0, w(), 20, (char *)"MainMenu");

    this->toolbar = new Toolbar(0, 20, this->w(), 30);
    this->toolbar->gui = this;
    this->sidebar = new Sidebar(0, this->toolbar->y() + this->toolbar->h(), toolbarWidth, this->h() - 115);

    // Create the frame mamanger
    this->frameMgr = new GLFrameManager(toolbarWidth, this->toolbar->y() + this->toolbar->h(), this->w()-toolbarWidth, this->h()-115, "");

    this->timeSlider = new Fl_Slider(35,this->h()-50,this->w()-35,20,"Time:" );
    this->timeSlider->type(FL_HOR_NICE_SLIDER);
    this->timeSlider->align(FL_ALIGN_LEFT);
    this->timeSlider->labelsize(10);
    this->timeSlider->callback(&Gui::TimeSliderCB, this);
    this->timeSlider->value(1.0);

    // Create the status bar
    this->statusbar = new StatusBar(0, this->h()-30, 
                         width, 30);

    this->statusbar->gui = this;
  }

  this->resizable(this->statusbar);
  this->resizable(this->frameMgr);

  this->end();
  this->show();

  Fl::check();
  Fl::wait(0.3);

  this->hasFocus = true;

  Fl::check();
  Fl::wait(0.3);

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

  if (node)
    this->frameMgr->Load( node->GetChild("frames") );
  else
    this->frameMgr->Load(NULL);
}

////////////////////////////////////////////////////////////////////////////////
// Save the gui params in xml format
void Gui::Save(std::string &prefix, std::ostream &stream)
{
  std::string p = prefix + "  ";

  this->sizeP->SetValue(Vector2<int>(this->GetWidth(), this->GetHeight()));
  this->posP->SetValue(Vector2<int>(this->x(), this->y()));

  stream << prefix <<  "<rendering:gui>\n";
  stream << prefix <<  "  " << *(this->sizeP) << "\n";
  stream << prefix <<  "  " << *(this->posP) << "\n";
  this->frameMgr->Save(p, stream);
  stream << prefix << "</rendering:gui>\n";
}

////////////////////////////////////////////////////////////////////////////////
/// Create the user camera's 
void Gui::CreateCameras()
{
  this->frameMgr->CreateCameras();
}

////////////////////////////////////////////////////////////////////////////////
/// Initalize the gui
void Gui::Init()
{
  this->frameMgr->Init();

  Pixmap p, mask;
  XpmCreatePixmapFromData(fl_display, RootWindow(fl_display, fl_screen),
      const_cast<char**>(icon_xpm), &p, &mask, NULL);

  XWMHints *hints;
  hints = XGetWMHints(fl_display, fl_xid(this));
  hints->icon_pixmap = p;
  hints->icon_mask = mask;
  hints->flags = IconPixmapHint | IconMaskHint;
  XSetWMHints(fl_display, fl_xid(this), hints);
}

////////////////////////////////////////////////////////////////////////////////
void Gui::Update()
{
  this->sidebar->Update();
  this->toolbar->Update();
  this->statusbar->Update();
  this->frameMgr->Update();
  
  if (!Simulator::Instance()->IsPaused())
    this->timeSlider->value(1.0);

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
    case FL_KEYUP:
      {
        if (Fl::event_key() == ' ')
        {
          Simulator::Instance()->SetPaused(!Simulator::Instance()->IsPaused() );
          return 1;
        }
      }

    case FL_SHORTCUT:
      if ( (Fl::event_state() & FL_CTRL) && Fl::event_key() == 113)
        Simulator::Instance()->SetUserQuit();
      break;
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

  if (Fl::event_key() == FL_Escape)
  {
    World::Instance()->SetSelectedEntity(NULL);
    return 1;
  }

  return Fl_Window::handle(event);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the average FPS
float Gui::GetAvgFPS() const
{
  return this->frameMgr->GetFPS();
}

////////////////////////////////////////////////////////////////////////////////
// Time slider CB
void Gui::TimeSliderCB( Fl_Widget * w, void *data)
{
  //Gui *self = (Gui*)(data);
  Fl_Slider *slider = (Fl_Slider*)(w);
  World::Instance()->GotoTime( slider->value() );
}
