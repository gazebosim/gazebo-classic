#ifndef OGREFRAMELISTENER_HH
#define OGREFRAMELISTENER_HH

#include <OgreFrameListener.h>
//#include <OgreEventListeners.h>
#include <Ogre.h>


namespace Ogre
{
  class EventProcessor;
  class InputReader;
  class MouseEvent;
  class RaySceneQuery;
}

namespace gazebo
{
  
  class OgreAdaptor;
  
  class OgreFrameListener : public Ogre::FrameListener//, public Ogre::WindowEventListener
  {
    public: OgreFrameListener();
    public: virtual ~OgreFrameListener();
  
    public: virtual bool frameStarted( const Ogre::FrameEvent &evt);
    public: virtual bool frameEnded( const Ogre::FrameEvent &evt);
  
    private: Ogre::Vector3 directionVec;
  
    private: float moveAmount;
    private: float moveScale;
    private: float rotateAmount;
  
  };

}
#endif
