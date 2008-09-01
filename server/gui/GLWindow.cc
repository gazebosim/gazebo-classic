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

#include "Model.hh"
#include <X11/keysym.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <FL/Fl_Menu_Item.H>
#include <FL/Fl_Menu_Bar.H>

#include <GL/glx.h>

#include "Param.hh"
#include "Entity.hh"
#include "OgreCamera.hh"
#include "OgreCreator.hh"
#include "Simulator.hh"
#include "Global.hh"
#include "GazeboMessage.hh"
#include "MainMenu.hh"
#include "CameraManager.hh"
#include "UserCamera.hh"
#include "World.hh"

#include "OgreHUD.hh"
#include "OgreAdaptor.hh"

#include "GLFrame.hh"
#include "GLWindow.hh"

using namespace gazebo;

GLWindow *GLWindow::activeWin = NULL;

////////////////////////////////////////////////////////////////////////////////
// Constructor
GLWindow::GLWindow( int x, int y, int w, int h, const std::string &label)
    : Fl_Gl_Window( x, y, w, h, "" )
{
  this->end();

  this->moveAmount = 1.0;
  this->moveScale = 1;
  this->rotateAmount = 0.5;

  this->directionVec.x = 0;
  this->directionVec.y = 0;
  this->directionVec.z = 0;
  this->leftMousePressed = false;
  this->rightMousePressed = false;
  this->middleMousePressed = false;


  this->keys.clear();

  if (activeWin == NULL)
    activeWin = this;
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
  this->mouseDrag = false;

  // Create the default camera.
  this->userCamera = new UserCamera( this );
  this->userCamera->Load(NULL);
  this->userCamera->Init();
  this->userCamera->RotatePitch( DTOR(30) );
  this->userCamera->Translate( Vector3(-5, 0, 1) );

  this->activeCamera = this->userCamera;
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
void GLWindow::flush()
{
  return;
}

////////////////////////////////////////////////////////////////////////////////
// Update function
void GLWindow::Update()
{
  if (this->activeCamera && this->activeCamera->GetUserMovable())
  {
    this->activeCamera->Translate( 
        this->directionVec * (Simulator::Instance()->GetRealTime() - this->lastUpdateTime) );
    this->directionVec.Set(0,0,0);
  }

  this->lastUpdateTime = Simulator::Instance()->GetRealTime();

  if (this->userCamera != this->activeCamera)
    this->activeCamera->UpdateCam();
  else
    this->userCamera->Update();
}


////////////////////////////////////////////////////////////////////////////////
/// Get a pointer to the camera
UserCamera *GLWindow::GetCamera() const
{
  return this->userCamera;
}

////////////////////////////////////////////////////////////////////////////////
// Get the average FPS for this window
float GLWindow::GetAvgFPS() const
{
  return this->activeCamera->GetAvgFPS();
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

  if (!this->mouseDrag)
  {
    Entity *ent = OgreAdaptor::Instance()->GetEntityAt(this->activeCamera, this->mousePos);

    if (ent)
    {
      Simulator::Instance()->SetSelectedEntity( ent );
    }
  }

  this->mouseDrag = false;
}

////////////////////////////////////////////////////////////////////////////////
/// Handle a mouse drag
void GLWindow::HandleMouseDrag()
{
  if (this->activeCamera && this->activeCamera->GetUserMovable())
  {
    Vector2<int> d = this->mousePos - this->prevMousePos;
    if (this->leftMousePressed)
    {
      this->activeCamera->RotateYaw(DTOR(-d.x * this->rotateAmount));
      this->activeCamera->RotatePitch(DTOR(d.y * this->rotateAmount));
    }
    else if (this->rightMousePressed)
    {
      Model *model = dynamic_cast<Model*>(Simulator::Instance()->GetSelectedEntity());
      if (model)
      {
        Pose3d pose = model->GetPose();
        pose.pos.y -= d.x * 0.05;
        pose.pos.x -= d.y * 0.05;
        model->SetPose(pose);
      }
      else
      {
        Vector2<int> d = this->mousePos - this->prevMousePos;
        this->directionVec.x = 0;
        this->directionVec.y =  d.x * this->moveAmount;
        this->directionVec.z =  d.y * this->moveAmount;
      }
    }
    else if (this->middleMousePressed)
    {
      Vector2<int> d = this->mousePos - this->prevMousePos;
      this->directionVec.x =  d.y * this->moveAmount;
      this->directionVec.y =  0;
      this->directionVec.z =  0;
    }
  }

  this->mouseDrag = true;
}

////////////////////////////////////////////////////////////////////////////////
// Handle mouse wheel movement
void GLWindow::HandleMouseWheel(int dx, int dy)
{
  Model *model = dynamic_cast<Model*>(Simulator::Instance()->GetSelectedEntity());
  if (model)
  {
    Pose3d pose = model->GetPose();
    pose.pos.z += dy * 0.05;
    model->SetPose(pose);
  }

}

////////////////////////////////////////////////////////////////////////////////
/// Handle a key press
void GLWindow::HandleKeyPress(int keyNum)
{
  std::map<int,int>::iterator iter;
  this->keys[keyNum] = 1;

  // loop through the keys to find the modifiers -- swh
  float moveAmount = this->moveAmount;
  for (iter = this->keys.begin(); iter!= this->keys.end(); iter++)
  {
    if (iter->second == 1)
    {
      switch (iter->first)
      {
        case FL_Control_L:
        case FL_Control_R:
          moveAmount = this->moveAmount * 10;
          break;
      }
    }
  }

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
}

////////////////////////////////////////////////////////////////////////////////
/// Handle a key release
void GLWindow::HandleKeyRelease(int keyNum)
{
  this->keys[keyNum] = 0;

  Simulator* sim = Simulator::Instance();

  // Handle all toggle keys
  switch (keyNum)
  {
    case FL_Escape:
      Simulator::Instance()->SetUserQuit();
      break;

    case '[':
      CameraManager::Instance()->IncActiveCamera();
      break;

    case ']':
      CameraManager::Instance()->DecActiveCamera();
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

  // Handle Events
  switch(event)
  {
    case FL_UNFOCUS:
    case FL_LEAVE:
      handled = true;
      break;

    case FL_FOCUS:
    case FL_ENTER: 
      activeWin = this;
      handled = true;
      break;

    case FL_CLOSE:
      Simulator::Instance()->SetUserQuit();
      handled = true;
      break;

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
      if (activeWin != this)
        activeWin->HandleKeyPress(Fl::event_key());
      else
        this->HandleKeyPress(Fl::event_key());
      handled = true;
      break;

    case FL_KEYUP:
      if (activeWin != this)
        activeWin->HandleKeyRelease(Fl::event_key());
      else
        this->HandleKeyRelease(Fl::event_key());
      handled = true;
      break;

    case FL_MOUSEWHEEL:
      this->HandleMouseWheel(Fl::event_dx(), Fl::event_dy());
      handled = true;
      break;

    default:
      break;
  }

  this->prevMousePos = this->mousePos;

  if (!handled)
    return Fl_Gl_Window::handle(event);
  else
    return 1;
}

////////////////////////////////////////////////////////////////////////////////
// Handle resizing the window
void GLWindow::resize(int x, int y, int w, int h)
{
  Fl_Window::resize(x,y,w,h);
  this->userCamera->Resize(w,h);

  this->redraw();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the visual info
XVisualInfo *GLWindow::GetVisualInfo() const
{
  return this->visual;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the display
Display *GLWindow::GetDisplay() const
{
  return this->display;
}


////////////////////////////////////////////////////////////////////////////////
/// Set the active camera
void GLWindow::SetActiveCamera( OgreCamera *camera )
{
   this->activeCamera = camera;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the style of the view = "front, left, top, user"
void GLWindow::SetViewStyle(std::string view)
{
  UserCamera *cam =  this->GetCamera();
  Pose3d pose = cam->GetWorldPose();

  if (view == "Top")
  {
    pose.rot.SetFromEuler( Vector3(0, DTOR(90), 0) );
  }
  else if (view == "Left")
  {
    pose.rot.SetFromEuler( Vector3(0, 0, DTOR(90) ) );
  }
  else if (view == "Front")
  {
    pose.rot.SetFromEuler( Vector3(0, 0,0) );
  }

  cam->SetWorldPose( pose );
}
