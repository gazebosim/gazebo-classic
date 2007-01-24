#include <CEGUISystem.h>
#include <OgreCEGUIRenderer.h>
#include <OgreKeyEvent.h>
#include <OgreMouseEvent.h>

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

  this->eventProcessor = new Ogre::EventProcessor();
  this->eventProcessor->initialise(this->ogreAdaptor->window);
  this->eventProcessor->startProcessingEvents();
  this->eventProcessor->addKeyListener(this);

  this->eventProcessor->addMouseListener( this );
  this->eventProcessor->addMouseMotionListener( this );

  this->inputDevice = this->eventProcessor->getInputReader();

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

void OgreFrameListener::keyClicked( Ogre::KeyEvent * /*e*/ )
{
}

void OgreFrameListener::keyPressed( Ogre::KeyEvent *e )
{
  switch (e->getKey())
  {
    case Ogre::KC_ESCAPE:
      userQuit = true;
      break;

    case Ogre::KC_UP:
    case Ogre::KC_W:
      this->directionVec.z -= this->moveAmount;
      break;

    case Ogre::KC_DOWN:
    case Ogre::KC_S:
      this->directionVec.z += this->moveAmount;
      break;

    case Ogre::KC_LEFT:
    case Ogre::KC_A:
     this->directionVec.x -= this->moveAmount;
      break;

    case Ogre::KC_RIGHT:
    case Ogre::KC_D:
      this->directionVec.x += this->moveAmount;
      break;

    case Ogre::KC_PGDOWN:
    case Ogre::KC_E:
      this->directionVec.y -= this->moveAmount;
      break;

    case Ogre::KC_PGUP:
    case Ogre::KC_Q:
      this->directionVec.y += this->moveAmount;
      break;
  }

}

void OgreFrameListener::keyReleased( Ogre::KeyEvent *e )
{
  switch (e->getKey())
  {
    case Ogre::KC_UP:
    case Ogre::KC_W:
      this->directionVec.z += this->moveAmount;
      break;

    case Ogre::KC_DOWN:
    case Ogre::KC_S:
      this->directionVec.z -= this->moveAmount;
      break;

    case Ogre::KC_LEFT:
    case Ogre::KC_A:
      this->directionVec.x += this->moveAmount;
      break;

    case Ogre::KC_RIGHT:
    case Ogre::KC_D:
      this->directionVec.x -= this->moveAmount;
      break;

    case Ogre::KC_PGDOWN:
    case Ogre::KC_E:
      this->directionVec.y += this->moveAmount;
      break;

    case Ogre::KC_PGUP:
    case Ogre::KC_Q:
      this->directionVec.y -= this->moveAmount;
      break;
  }
}


void OgreFrameListener::mouseDragged(Ogre::MouseEvent* e)
{
  Camera *camera;

  if ((camera = CameraManager::Instance()->GetActiveCamera()))
  {
    camera->RotateYaw(-e->getRelX( ) * this->rotateAmount);
    camera->RotatePitch(-e->getRelY() * this->rotateAmount);
  }
}

void OgreFrameListener::mouseMoved(Ogre::MouseEvent *e)
{
   // Update CEGUI with the mouse motion
/*   CEGUI::System::getSingleton().injectMouseMove(
       e->getRelX() * this->ogreAdaptor->guiRenderer->getWidth(), 
       e->getRelY() * this->ogreAdaptor->guiRenderer->getHeight());
       */
}

void OgreFrameListener::mousePressed(Ogre::MouseEvent* e)
{
  CEGUI::MouseCursor::getSingleton().hide();
}

void OgreFrameListener::mouseReleased(Ogre::MouseEvent* e)
{
  CEGUI::MouseCursor::getSingleton().show();
}
