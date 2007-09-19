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
/* Desc: Gazebo input handler
 * Author: Nate Koenig
 * Date: 17 Sep 2007
 * SVN: $Id:$
 */

#include <fltk/run.h>
#include <X11/keysym.h>

#include "CameraSensor.hh"
#include "Global.hh"
#include "OgreHUD.hh"
#include "CameraManager.hh"
#include "InputEvent.hh"
#include "InputHandler.hh"

#if FL_MAJOR_VERSION == 2
using namespace fltk;
#endif

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
InputHandler::InputHandler()
{
  this->moveAmount = 0.05;
  this->moveScale = 1;
  this->rotateAmount = 1;

  this->directionVec.x = 0;
  this->directionVec.y = 0;
  this->directionVec.z = 0;
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
InputHandler::~InputHandler()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Handle an input event
void InputHandler::HandleEvent(const InputEvent *event)
{
  switch( event->GetType() )
  {
    case InputEvent::MOUSE_PRESS:
      if (event->GetMouseButton() == InputEvent::LEFT_MOUSE)
        this->leftMousePressed = true;
      if (event->GetMouseButton() == InputEvent::RIGHT_MOUSE)
        this->rightMousePressed = true;
      if (event->GetMouseButton() == InputEvent::MIDDLE_MOUSE)
        this->middleMousePressed = true;
      this->prevMousePos = event->GetMousePos();
      break;

    case InputEvent::MOUSE_RELEASE:
      if (event->GetMouseButton() == InputEvent::LEFT_MOUSE)
        this->leftMousePressed = false;
      if (event->GetMouseButton() == InputEvent::RIGHT_MOUSE)
        this->rightMousePressed = false;
      if (event->GetMouseButton() == InputEvent::MIDDLE_MOUSE)
        this->middleMousePressed = false;
      this->prevMousePos = event->GetMousePos();
      break;

    case InputEvent::KEY_PRESS:
      this->HandleKeyPress(event);
      break;

    case InputEvent::KEY_RELEASE:
      this->HandleKeyRelease(event);
      break;

    case InputEvent::MOUSE_DRAG:
      this->HandleDrag(event);
      break;

    default:
      break;
  }

  this->prevMousePos = event->GetMousePos();
}

////////////////////////////////////////////////////////////////////////////////
// Handle key press
void InputHandler::HandleKeyPress( const InputEvent *event )
{
  this->keys[event->GetKey()] = 1;
}

////////////////////////////////////////////////////////////////////////////////
// Handle key release
bool InputHandler::HandleKeyRelease( const InputEvent *event )
{
  //printf("Release[%d]\n",event->GetKey());

  this->keys[event->GetKey()] = 0;

  // Handle all toggle keys
  switch (event->GetKey())
  {
    case XK_t:
      if (Global::userPause)
        Global::userPause = false;
      Global::userStep = !Global::userStep;
      Global::userStepInc = false;
      break;

    case XK_h:
      OgreHUD::Instance()->ToggleHelp();
      break;

    case XK_space:
      if (Global::userStep)
      {
        Global::userStepInc = true;
      }
      else
        Global::userPause = !Global::userPause;
      break;

    case XK_Escape:
      Global::userQuit = true;
      break;

    case XK_bracketleft:
      CameraManager::Instance()->IncActiveCamera();
      break;

    case XK_bracketright:
      CameraManager::Instance()->DecActiveCamera();
      break;

    case XK_Tab:
      OgreHUD::Instance()->ToggleVisible();
      break;
  }
  return true;
}


void InputHandler::Update()
{
  CameraSensor *camera = CameraManager::Instance()->GetActiveCamera();
  std::map<int,int>::iterator iter;

  for (iter = this->keys.begin(); iter!= this->keys.end(); iter++)
  {
    if (iter->second == 1)
    {
      switch (iter->first)
      {
        case XK_Up:
        case XK_w:
          this->directionVec.z -= this->moveAmount;
          break;

        case XK_Down:
        case XK_s:
          this->directionVec.z += this->moveAmount;
          break;

        case XK_Left:
        case XK_a:
          this->directionVec.x -= this->moveAmount;
          break;

        case XK_Right:
        case XK_d:
          this->directionVec.x += this->moveAmount;
          break;

        case XK_Page_Down:
        case XK_e:
          this->directionVec.y -= this->moveAmount;
          break;

        case XK_Page_Up:
        case XK_q:
          this->directionVec.y += this->moveAmount;
          break;

        default:
          break;
      }
    }
  }
  camera->Translate(this->directionVec);
  this->directionVec.Set(0,0,0);
}

////////////////////////////////////////////////////////////////////////////////
// Handle a drag event
void InputHandler::HandleDrag( const InputEvent *event )
{
  CameraSensor *camera = CameraManager::Instance()->GetActiveCamera();

  if (camera)
  {
    if (this->leftMousePressed)
    {
      Vector2<int> d = event->GetMousePos() - this->prevMousePos;
      camera->RotateYaw(-d.x * this->rotateAmount);
      camera->RotatePitch(-d.y * this->rotateAmount);
    }
  }
}
