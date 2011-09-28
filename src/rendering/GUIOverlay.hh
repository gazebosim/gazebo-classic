#ifndef GUI_OVERLAY
#define GUI_OVERLAY

#include <string>
#include <map>

#include "common/MouseEvent.hh"
#include "gazebo_config.h"
#include "math/MathTypes.hh"

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

      public: void CreateWindow( const std::string &_type, 
                                 const std::string &_name,
                                 const std::string &_parent,
                                 const math::Vector2d &_position, 
                                 const math::Vector2d &_size,
                                 const std::string &_text);

      public: bool HandleMouseEvent( const common::MouseEvent &_evt);


#ifdef HAVE_CEGUI
      private: CEGUI::OgreRenderer *guiRenderer;
      private: std::map<std::string, CEGUI::Window*> windows;
#endif

    };
  }
}

#endif
