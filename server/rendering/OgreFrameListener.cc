#include <CEGUISystem.h>
#include <OgreCEGUIRenderer.h>
#include <OgreWindowEventUtilities.h>

#include <OIS.h>
#include <OISEvents.h>
#include <OISInputManager.h>
#include <OISMouse.h>
#include <OISKeyboard.h>

#include "Camera.hh"
#include "CameraManager.hh"
#include "OgreAdaptor.hh"
#include "OgreFrameListener.hh"

using namespace gazebo;

extern bool userQuit;

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
  Camera *camera;

  if ((camera = CameraManager::Instance()->GetActiveCamera()))
  {
    camera->Translate(this->directionVec * evt.timeSinceLastFrame );
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
   // Update CEGUI with the mouse motion
   /*CEGUI::System::getSingleton().injectMouseMove(
       e.state.X.rel * this->ogreAdaptor->guiRenderer->getWidth(), 
       e.state.Y.rel * this->ogreAdaptor->guiRenderer->getHeight());
       */

  Camera *camera;

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
  //CEGUI::MouseCursor::getSingleton().hide();

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
//  CEGUI::MouseCursor::getSingleton().show();

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
