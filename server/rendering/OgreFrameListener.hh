#ifndef OGREFRAMELISTENER_HH
#define OGREFRAMELISTENER_HH

#include <OgreFrameListener.h>
//#include <OgreEventListeners.h>
#include <Ogre.h>
#include <OIS/OIS.h>


namespace Ogre
{
  class EventProcessor;
  class InputReader;
  class MouseEvent;
}

class OgreAdaptor;

class OgreFrameListener : public Ogre::FrameListener, public Ogre::WindowEventListener, public OIS::KeyListener, public OIS::MouseListener
{
  public: OgreFrameListener( OgreAdaptor *ogreAdaptor );
  public: virtual ~OgreFrameListener();

  public: virtual bool frameStarted( const Ogre::FrameEvent &evt);
  public: virtual bool frameEnded( const Ogre::FrameEvent &evt);

  public: virtual bool keyPressed( const OIS::KeyEvent &e );
  public: virtual bool keyReleased( const OIS::KeyEvent &e );

  // MouseDragged
  public: bool mouseMoved(const OIS::MouseEvent &e);
  public: bool mouseDragged(const OIS::MouseEvent &e);

  // MouseListener
  public: bool mousePressed(const OIS::MouseEvent &e, OIS::MouseButtonID id);
  public: bool mouseReleased(const OIS::MouseEvent &e, OIS::MouseButtonID id);

  private: Ogre::Vector3 directionVec;

  private: OgreAdaptor *ogreAdaptor;
  private: float moveAmount;
  private: float rotateAmount;

  private: Ogre::EventProcessor *eventProcessor;

  private: OIS::InputManager *inputManager;
  private: OIS::Mouse *mMouse;
  private: OIS::Keyboard *mKeyboard;
};

#endif
