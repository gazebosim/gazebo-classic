#ifndef OGREFRAMELISTENER_HH
#define OGREFRAMELISTENER_HH

#include <OgreFrameListener.h>
#include <OgreEventListeners.h>
#include <Ogre.h>


namespace Ogre
{
  class EventProcessor;
  class InputReader;
  class MouseEvent;
}

class OgreAdaptor;

class OgreFrameListener : public Ogre::FrameListener, public Ogre::KeyListener, public Ogre::MouseListener, public Ogre::MouseMotionListener
{
  public: OgreFrameListener( OgreAdaptor *ogreAdaptor );
  public: virtual ~OgreFrameListener();

  public: virtual bool frameStarted( const Ogre::FrameEvent &evt);
  public: virtual bool frameEnded( const Ogre::FrameEvent &evt);

  public: virtual void keyClicked( Ogre::KeyEvent *e );
  public: virtual void keyPressed( Ogre::KeyEvent *e );
  public: virtual void keyReleased( Ogre::KeyEvent *e );

  // MouseDragged
  public: void mouseMoved(Ogre::MouseEvent *e);
  public: void mouseDragged(Ogre::MouseEvent* e);

  // MouseListener
  public: void mouseClicked(Ogre::MouseEvent* /*e*/){}
  public: void mouseEntered(Ogre::MouseEvent* /*e*/) {}
  public: void mouseExited(Ogre::MouseEvent* /*e*/) {}
  public: void mouseReleased(Ogre::MouseEvent* e);
  public: void mousePressed(Ogre::MouseEvent* e);

  private: Ogre::Vector3 directionVec;

  private: OgreAdaptor *ogreAdaptor;
  private: float moveAmount;
  private: float rotateAmount;

  private: Ogre::EventProcessor *eventProcessor;
  private: Ogre::InputReader *inputDevice;
};

#endif
