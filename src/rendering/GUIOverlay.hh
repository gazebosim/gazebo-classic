#ifndef GUI_OVERLAY
#define GUI_OVERLAY

#include <string>

#include "common/MouseEvent.hh"
#include "common/Events.hh"

#include "gazebo_config.h"
#include "msgs/MessageTypes.hh"
#include "math/MathTypes.hh"
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
      public: CEGUI::Window *LoadLayout( const std::string &_filename );

      private: void OnConfig( const boost::shared_ptr<msgs::GUIOverlayConfig const> &_msg);

      private: void PreRender();

#ifdef HAVE_CEGUI
      private: CEGUI::OgreRenderer *guiRenderer;
#endif

      private: transport::NodePtr node;  
      private: transport::SubscriberPtr configSub;  

      private: std::vector<event::ConnectionPtr> connections;
      private: boost::shared_ptr<msgs::GUIOverlayConfig const> configMsg;
    };
  }
}

#endif
