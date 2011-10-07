#ifndef GUI_OVERLAY
#define GUI_OVERLAY

#include <string>

#include "common/MouseEvent.hh"
#include "common/Events.hh"

#include "gazebo_config.h"
#include "math/MathTypes.hh"

#include "rendering/RenderTypes.hh"
#include "msgs/MessageTypes.hh"
#include "transport/TransportTypes.hh"

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

      public: bool IsInitialized();

      public: void CreateWindow( const std::string &_type, 
                                 const std::string &_name,
                                 const std::string &_parent,
                                 const math::Vector2d &_position, 
                                 const math::Vector2d &_size,
                                 const std::string &_text);

      public: bool HandleMouseEvent( const common::MouseEvent &_evt);

      /// \brief Load a CEGUI layout file
      public: void LoadLayout( const std::string &_filename );

      public: bool AttachCameraToImage(CameraPtr &_camera, 
                  const std::string &_windowName);

      private: void PreRender();

      /// Load a CEGUI layout file
      private: CEGUI::Window *LoadLayoutImpl( const std::string &_filename );

#ifdef HAVE_CEGUI_OGRE
      private: CEGUI::OgreRenderer *guiRenderer;
#endif

      private: std::vector<event::ConnectionPtr> connections;
      private: std::string layoutFilename;
    };
  }
}

#endif
