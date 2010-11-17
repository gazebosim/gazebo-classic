#ifndef WINDOWMANAGER_HH
#define WINDOWMANAGER_HH

#include "SingletonT.hh"
#include <string>
#include <vector>

namespace Ogre
{
  class RenderWindow;
}

namespace gazebo
{
  class RenderControl;
  class Camera;

  class WindowManager : public SingletonT<WindowManager>
  {
    public: WindowManager();
    public: virtual ~WindowManager();

    public: int CreateWindow( RenderControl *control );

    public: int CreateWindow( std::string ogreHandle, 
                              unsigned int width, 
                              unsigned int height );


    /// \brief Attach a camera to a window
    public: void SetCamera( int windowId, Camera *camera);


    private: std::vector<Ogre::RenderWindow *> windows;

    private: static unsigned int windowCounter;
    
    private: friend class DestroyerT<WindowManager>;
    private: friend class SingletonT<WindowManager>;
  };
}
#endif
