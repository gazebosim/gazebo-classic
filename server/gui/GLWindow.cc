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
/* Desc: GL Window
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 * SVN: $Id:$
 */

#include "CameraSensor.hh"
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <FL/Fl_Menu_Item.H>
#include <FL/Fl_Menu_Bar.H>

#include <GL/glx.h>

#include "Simulator.hh"
#include "Global.hh"
#include "GazeboMessage.hh"
#include "MainMenu.hh"
#include "CameraManager.hh"

#include "OgreHUD.hh"

#include "GLWindow.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
GLWindow::GLWindow(int x, int y, int w, int h, const std::string &label) 
  : Fl_Gl_Window( x, y, w, h, label.c_str() )
{

  this->end();

  this->moveAmount = 1.0;
  this->moveScale = 1;
  this->rotateAmount = 0.5;

  this->directionVec.x = 0;
  this->directionVec.y = 0;
  this->directionVec.z = 0;

  this->keys.clear();

}


////////////////////////////////////////////////////////////////////////////////
// Destructor
GLWindow::~GLWindow()
{
}

////////////////////////////////////////////////////////////////////////////////
// Init
void GLWindow::Init()
{
  this->show();

  // Must have the next two lines right here!!!!
  this->make_current();
  this->valid(1);

  this->display = fl_display;
  this->visual = fl_visual;
  this->colormap = fl_colormap;
  this->windowId = Fl_X::i(this)->xid;

  Fl_Window::show();

}

////////////////////////////////////////////////////////////////////////////////
/// Get the width of the gui's rendering window
unsigned int GLWindow::GetWidth() const
{
  return this->w();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the height of the gui's rendering window
unsigned int GLWindow::GetHeight() const
{
  return this->h();
}

////////////////////////////////////////////////////////////////////////////////
// Update function
void GLWindow::Update()
{
  CameraSensor *camera = CameraManager::Instance()->GetActiveCamera();
  std::map<int,int>::iterator iter;

  for (iter = this->keys.begin(); iter!= this->keys.end(); iter++)
  {
    if (iter->second == 1)
    {
      switch (iter->first)
      {
        case '=':
        case '+':
          this->moveAmount *= 2;
          break;

        case '-':
        case '_':
          this->moveAmount *= 0.5;
          break;

        case XK_Up:
        case XK_w:
          this->directionVec.x += this->moveAmount;
          break;

        case XK_Down:
        case XK_s:
          this->directionVec.x -= this->moveAmount;
          break;

        case XK_Left:
        case XK_a:
          this->directionVec.y += this->moveAmount;
          break;

        case XK_Right:
        case XK_d:
          this->directionVec.y -= this->moveAmount;
          break;

        case XK_Page_Down:
        case XK_e:
          this->directionVec.z += this->moveAmount;
          break;

        case XK_Page_Up:
        case XK_q:
          this->directionVec.z -= this->moveAmount;
          break;

        default:
          break;
      }
    }
  }

  camera->Translate( this->directionVec * (Simulator::Instance()->GetRealTime() - this->lastUpdateTime) );
  this->directionVec.Set(0,0,0);

  this->lastUpdateTime = Simulator::Instance()->GetRealTime();
}

void GLWindow::flush()
{
  return;
}

////////////////////////////////////////////////////////////////////////////////
/// Handle a mouse button push
void GLWindow::HandleMousePush()
{
  // Get the mouse button that was pressed (if one was pressed)
  switch (Fl::event_button())
  {
    case FL_LEFT_MOUSE:
      this->leftMousePressed = true;
      break;

    case FL_RIGHT_MOUSE:
      this->rightMousePressed = true;
      break;

    case FL_MIDDLE_MOUSE:
      this->middleMousePressed = true;
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Handle a mouse button release
void GLWindow::HandleMouseRelease()
{
  // Get the mouse button that was pressed (if one was pressed)
  switch (Fl::event_button())
  {
    case FL_LEFT_MOUSE:
      this->leftMousePressed = false;
      break;

    case FL_RIGHT_MOUSE:
      this->rightMousePressed = false;
      break;

    case FL_MIDDLE_MOUSE:
      this->middleMousePressed = false;
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Handle a mouse drag
void GLWindow::HandleMouseDrag()
{
  CameraSensor *camera = CameraManager::Instance()->GetActiveCamera();

  if (camera)
  {
    if (this->leftMousePressed)
    {
      Vector2<int> d = this->mousePos - this->prevMousePos;
      camera->RotateYaw(-d.x * this->rotateAmount);
      camera->RotatePitch(d.y * this->rotateAmount);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Handle a key press
void GLWindow::HandleKeyPress()
{
  this->keys[Fl::event_key()] = 1;
}

////////////////////////////////////////////////////////////////////////////////
/// Handle a key release
void GLWindow::HandleKeyRelease()
{
  this->keys[Fl::event_key()] = 0;

  Simulator* sim=Simulator::Instance();
  // Handle all toggle keys
  switch (Fl::event_key())
  {
    case 't':
      if (sim->GetUserPause())
        sim->SetUserPause(false);
      sim->SetUserStep( !sim->GetUserStep() );
      sim->SetUserStepInc( false );
      break;

    case 'h':
      //OgreHUD::Instance()->ToggleHelp();
      break;

    case ' ':
      
      if (sim->GetUserStep())
      {
        sim->SetUserStepInc( true );
      }
      else
        sim->SetUserPause( !sim->GetUserPause() );
      break;

    case FL_Escape:
      Simulator::Instance()->SetUserQuit();
      break;

    case '[':
      CameraManager::Instance()->IncActiveCamera();
      break;

    case ']':
      CameraManager::Instance()->DecActiveCamera();
      break;

    case FL_Tab:
      //OgreHUD::Instance()->ToggleVisible();
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Handle events
int GLWindow::handle(int event)
{
  bool handled = false;

  // Get the mouse position
  this->mousePos = Vector2<int>( Fl::event_x(), Fl::event_y() );

  // Set the type of the event
  switch (event)
  {
    case FL_ENTER:
    case FL_LEAVE:
    case FL_DEACTIVATE:
    case FL_HIDE:
      //this->inputHandler->ClearEvents();
      return 0;

    case FL_CLOSE:
      Simulator::Instance()->SetUserQuit();
      return 0;

    case FL_PUSH:
      this->HandleMousePush();
      handled = true;
      break;

    case FL_RELEASE:
      this->HandleMouseRelease();
      handled = true;
      break;

    case FL_DRAG:
      this->HandleMouseDrag();
      handled = true;
      break;

    case FL_SHORTCUT:
    case FL_KEYDOWN:
      this->HandleKeyPress();
      handled = true;
      break;

    case FL_KEYUP:
      this->HandleKeyRelease();
      handled = true;
      break;
  }

  this->prevMousePos = this->mousePos;

  if (!handled)
    return Fl_Gl_Window::handle(event);
  else
    return 1;
}
