#ifndef GUI_OVERLAY
#define GUI_OVERLAY


namespace Ogre
{
  class RenderTarget;
}

namespace CEGUI
{
  class OgreRenderer;
  class Window;
}

namespace gazebo
{
  namespace rendering
  {
    class GUIOverlay
    {
      public: GUIOverlay();
      public: virtual ~GUIOverlay();

      public: void Init( Ogre::RenderTarget *_renderTarget );

      private: CEGUI::OgreRenderer *guiRenderer;
      private: CEGUI::Window *rootWindow;
    };
  }
}

#endif
