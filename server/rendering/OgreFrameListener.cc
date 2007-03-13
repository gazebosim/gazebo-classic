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

extern bool userQuit;

OgreFrameListener::OgreFrameListener( OgreAdaptor *ogreAdaptor )
{
  this->ogreAdaptor = ogreAdaptor;
  this->moveAmount = 5;
  this->rotateAmount = 72;

  OIS::ParamList pl;
  size_t windowHnd = 0;
  std::ostringstream windowHndStr;

  this->ogreAdaptor->window->getCustomAttribute("WINDOW",&windowHnd);
  windowHndStr << windowHnd;
  pl.insert(std::make_pair(std::string("WINDOW"), windowHndStr.str()));

  // Create the input manager
  this->inputManager = OIS::InputManager::createInputSystem(pl);

  bool bufferedKeys = false;
  bool bufferedMouse = false;

  // Create the devices
  this->mKeyboard = static_cast<OIS::Keyboard*>(this->inputManager->createInputObject( OIS::OISKeyboard, bufferedKeys));
  this->mMouse = static_cast<OIS::Mouse*>(this->inputManager->createInputObject( OIS::OISMouse, bufferedMouse));

  Ogre::WindowEventUtilities::addWindowEventListener(this->ogreAdaptor->window, this);

  /*this->eventProcessor = new Ogre::EventProcessor();
  this->eventProcessor->initialise(this->ogreAdaptor->window);
  this->eventProcessor->startProcessingEvents();
  this->eventProcessor->addKeyListener(this);

  this->eventProcessor->addMouseListener( this );
  this->eventProcessor->addMouseMotionListener( this );

  this->inputDevice = this->eventProcessor->getInputReader();
  */

  this->directionVec[0] = 0;
  this->directionVec[1] = 0;
  this->directionVec[2] = 0;
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
  }

  return true;
}


/*void OgreFrameListener::mouseDragged(Ogre::MouseEvent* e)
{
  Camera *camera;

  if ((camera = CameraManager::Instance()->GetActiveCamera()))
  {
    camera->RotateYaw(-e->getRelX( ) * this->rotateAmount);
    camera->RotatePitch(-e->getRelY() * this->rotateAmount);
  }
}*/

bool OgreFrameListener::mouseMoved(const OIS::MouseEvent &e)
{
   // Update CEGUI with the mouse motion
   CEGUI::System::getSingleton().injectMouseMove(
       e.state.X.rel * this->ogreAdaptor->guiRenderer->getWidth(), 
       e.state.Y.rel * this->ogreAdaptor->guiRenderer->getHeight());

   return true;
}

bool OgreFrameListener::mousePressed(const OIS::MouseEvent &e, OIS::MouseButtonID id)
{
  CEGUI::MouseCursor::getSingleton().hide();

  return true;
}

bool OgreFrameListener::mouseReleased(const OIS::MouseEvent &e, OIS::MouseButtonID id)
{
  CEGUI::MouseCursor::getSingleton().show();

  return true;
}
