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
/* Desc: OGRE frame listener
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 * CVS: $Id$
 */

#include <OgreWindowEventUtilities.h>

#include <OIS.h>
#include <OISEvents.h>
#include <OISInputManager.h>
#include <OISMouse.h>
#include <OISKeyboard.h>

#include "Global.hh"
#include "Pose3d.hh"
#include "OgreHUD.hh"
#include "CameraSensor.hh"
#include "CameraManager.hh"
#include "OgreAdaptor.hh"
#include "OgreFrameListener.hh"

using namespace gazebo;

extern bool userQuit;
extern bool userPause;

OgreFrameListener::OgreFrameListener()
{
  this->moveAmount = 1;
  this->rotateAmount = 1;

  OIS::ParamList pl;
  size_t windowHnd = 0;
  std::ostringstream windowHndStr;

  OgreAdaptor::Instance()->window->getCustomAttribute("WINDOW",&windowHnd);
  windowHndStr << windowHnd;
  pl.insert(std::make_pair(std::string("WINDOW"), windowHndStr.str()));

  // Allow the cursor to be showen, and prevent Gazebo from hogging the
  // cursor.
  pl.insert(std::make_pair(std::string("x11_mouse_grab"), 
                           std::string("false")));
  pl.insert(std::make_pair(std::string("x11_mouse_hide"), 
                           std::string("false")));
  pl.insert(std::make_pair(std::string("x11_keyboard_grab"), 
                           std::string("false")));
  pl.insert(std::make_pair(std::string("XAutoRepeatOn"), 
                           std::string("true")));

  // Create the input manager
  this->inputManager = OIS::InputManager::createInputSystem(pl);

  bool bufferedKeys = true;
  bool bufferedMouse = true;

  // Create the devices
  this->mKeyboard = static_cast<OIS::Keyboard*>(this->inputManager->createInputObject( OIS::OISKeyboard, bufferedKeys));
  this->mKeyboard->setEventCallback(this);

  this->mMouse = static_cast<OIS::Mouse*>(this->inputManager->createInputObject( OIS::OISMouse, bufferedMouse));
  this->mMouse->setEventCallback(this);

  unsigned int width, height, depth;
  int top, left;
  OgreAdaptor::Instance()->window->getMetrics(width, height, depth, left, top);
  const OIS::MouseState &mouseState = this->mMouse->getMouseState();
  mouseState.width = width;
  mouseState.height = height;

  this->directionVec[0] = 0;
  this->directionVec[1] = 0;
  this->directionVec[2] = 0;

  this->leftPressed = false;
  this->rightPressed = false;
  this->middlePressed = false;
}

OgreFrameListener::~OgreFrameListener()
{
}

bool OgreFrameListener::frameStarted( const Ogre::FrameEvent &evt)
{
  CameraSensor *camera;

  if ((camera = CameraManager::Instance()->GetActiveCamera()))
  {
    Ogre::Vector3 tmp =this->directionVec * evt.timeSinceLastFrame;
    camera->Translate(Vector3(tmp.x, tmp.y, tmp.z));

    OgreHUD::Instance()->SetCamera(camera);
  }

  this->mKeyboard->capture();
  this->mMouse->capture();

  return true;
}

bool OgreFrameListener::frameEnded( const Ogre::FrameEvent &/*evt*/) 
{
  return true;
}

bool OgreFrameListener::keyPressed( const OIS::KeyEvent &e )
{
  switch (e.key)
  {
    case OIS::KC_SPACE:
      userPause = !userPause;
      break;

    case OIS::KC_ESCAPE:
      userQuit = true;
      break;

    case OIS::KC_UP:
    case OIS::KC_W:
      this->directionVec.z -= this->moveAmount;
      break;

    case OIS::KC_DOWN:
    case OIS::KC_S:
      this->directionVec.z += this->moveAmount;
      break;

    case OIS::KC_LEFT:
    case OIS::KC_A:
     this->directionVec.x -= this->moveAmount;
      break;

    case OIS::KC_RIGHT:
    case OIS::KC_D:
      this->directionVec.x += this->moveAmount;
      break;

    case OIS::KC_PGDOWN:
    case OIS::KC_E:
      this->directionVec.y -= this->moveAmount;
      break;

    case OIS::KC_PGUP:
    case OIS::KC_Q:
      this->directionVec.y += this->moveAmount;
      break;

    case OIS::KC_LBRACKET:
      CameraManager::Instance()->IncActiveCamera();
      break;

    case OIS::KC_RBRACKET:
      CameraManager::Instance()->DecActiveCamera();
      break;

    default:
      break;
  }

  return true;
}

bool OgreFrameListener::keyReleased( const OIS::KeyEvent &e )
{
  switch (e.key)
  {
    case OIS::KC_UP:
    case OIS::KC_W:
      this->directionVec.z += this->moveAmount;
      break;

    case OIS::KC_DOWN:
    case OIS::KC_S:
      this->directionVec.z -= this->moveAmount;
      break;

    case OIS::KC_LEFT:
    case OIS::KC_A:
      this->directionVec.x += this->moveAmount;
      break;

    case OIS::KC_RIGHT:
    case OIS::KC_D:
      this->directionVec.x -= this->moveAmount;
      break;

    case OIS::KC_PGDOWN:
    case OIS::KC_E:
      this->directionVec.y += this->moveAmount;
      break;

    case OIS::KC_PGUP:
    case OIS::KC_Q:
      this->directionVec.y -= this->moveAmount;
      break;
    default:
      break;
  }

  return true;
}

bool OgreFrameListener::mouseMoved(const OIS::MouseEvent &e)
{
  CameraSensor *camera;

  if ((camera = CameraManager::Instance()->GetActiveCamera()))
  {
    if (this->leftPressed)
    {
      camera->RotateYaw(-e.state.X.rel * this->rotateAmount);
      camera->RotatePitch(-e.state.Y.rel * this->rotateAmount);
    }
  }

   return true;
}

bool OgreFrameListener::mousePressed(const OIS::MouseEvent & /*e*/, OIS::MouseButtonID id)
{
  switch (id)
  {
    case OIS::MB_Left:
      this->leftPressed = true;
      break;
    case OIS::MB_Right:
      this->rightPressed = true;
      break;
    case OIS::MB_Middle:
      this->middlePressed = true;
      break;
    default:
      break;
  }

  return true;
}

bool OgreFrameListener::mouseReleased(const OIS::MouseEvent & /*e*/, OIS::MouseButtonID id)
{

  switch (id)
  {
    case OIS::MB_Left:
      this->leftPressed = false;
      break;
    case OIS::MB_Right:
      this->rightPressed = false;
      break;
    case OIS::MB_Middle:
      this->middlePressed = false;
      break;
    default:
      break;
  }

  return true;
}
