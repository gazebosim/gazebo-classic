#ifndef GUI_OVERLAY
#define GUI_OVERLAY


namespace CEGUI
{
  class OgreCEGuiRenderer;
}

namespace gazebo
{
  namespace rendering
  {
    class GuiOverlay
    {
      public: GuiOverlay();
      public: virtual ~GuiOverlay();

      public: void Init( Ogre::RenderWindow *_renderWindow );

      private: CEGUI::OgreCEGUIRenderer *guiRenderer;
      private: CEGUI::System *guiSystem;
    };
  }
}

#endif
