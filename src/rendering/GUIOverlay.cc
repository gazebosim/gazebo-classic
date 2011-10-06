#include "rendering/GUIOverlay.hh"
#include "common/Console.hh"

#ifdef HAVE_CEGUI
#include "CEGUI/CEGUI.h"
#include "CEGUI/RendererModules/Ogre/CEGUIOgreRenderer.h"
#endif

#include "msgs/msgs.h"
#include "transport/Transport.hh"
#include "transport/Node.hh"
#include "math/Vector2d.hh"

using namespace gazebo;
using namespace rendering;

GUIOverlay::GUIOverlay()
{

  this->connections.push_back( event::Events::ConnectPreRenderSignal( boost::bind(&GUIOverlay::PreRender, this) ) );
}

GUIOverlay::~GUIOverlay()
{
#ifdef HAVE_CEGUI
  CEGUI::OgreRenderer::destroySystem();
#endif
}

void GUIOverlay::Init( Ogre::RenderTarget *_renderTarget )
{
#ifdef HAVE_CEGUI
  this->guiRenderer = &CEGUI::OgreRenderer::bootstrapSystem(*_renderTarget);

  CEGUI::Imageset::setDefaultResourceGroup("Imagesets");
  CEGUI::Font::setDefaultResourceGroup("Fonts");
  CEGUI::Scheme::setDefaultResourceGroup("Schemes");
  CEGUI::WidgetLookManager::setDefaultResourceGroup("LookNFeel");
  CEGUI::WindowManager::setDefaultResourceGroup("Layouts");

  CEGUI::SchemeManager::getSingleton().create("TaharezLook.scheme");
  CEGUI::SchemeManager::getSingleton().create("VanillaSkin.scheme");
  CEGUI::FontManager::getSingleton().create("DejaVuSans-10.font");
  //CEGUI::System::getSingleton().setDefaultMouseCursor("TaharezLook", "MouseArrow");

  // clearing this queue actually make sure it's created
  this->guiRenderer->getDefaultRenderingRoot().clearGeometry(CEGUI::RQ_OVERLAY);

  // Create a root window, and set it as the root window
  CEGUI::Window *rootWindow  = CEGUI::WindowManager::getSingleton().createWindow("DefaultWindow", "root");
  CEGUI::System::getSingleton().setGUISheet( rootWindow );
#endif

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->configSub = this->node->Subscribe("~/gui_overlay_config",
      &GUIOverlay::OnConfig, this);
}

void GUIOverlay::CreateWindow( const std::string &_type, 
                               const std::string &_name,
                               const std::string &_parent,
                               const math::Vector2d &_position, 
                               const math::Vector2d &_size,
                               const std::string &_text)
{
#ifdef HAVE_CEGUI

  CEGUI::Window *parent = CEGUI::WindowManager::getSingleton().getWindow(_parent);
  CEGUI::Window *window = CEGUI::WindowManager::getSingleton().createWindow(_type, _name);

  parent->addChildWindow( window );

  window->setPosition( CEGUI::UVector2( CEGUI::UDim(_position.x, 0), 
                                             CEGUI::UDim(_position.y, 0) ) );

  window->setSize( CEGUI::UVector2( CEGUI::UDim(_size.x, 0), 
                                         CEGUI::UDim(_size.y, 0) ) );
  window->setText( _text );
#endif
}

bool GUIOverlay::HandleMouseEvent( const common::MouseEvent &/*_evt*/)
{
  bool result = false;
#ifdef HAVE_CEGUI
  //CEGUI::System *system = CEGUI::System::getSingletonPtr();
  //result = result || system->injectMousePosition( _evt.pos.x, _evt.pos.y);
/*
  if (_evt. == common::MouseEvent::DOWN)
    result = result || system->injectMouseButtonDown( CEGUI::LeftButton);
  else if (_evt.left == common::MouseEvent::UP)
    result = result || system->injectMouseButtonUp( CEGUI::LeftButton);

  if (_evt.right == common::MouseEvent::DOWN)
    result = result || system->injectMouseButtonDown( CEGUI::RightButton);
  else if (_evt.right == common::MouseEvent::UP)
    result = result ||system->injectMouseButtonUp( CEGUI::RightButton);
    */
#endif

  return result;
}

bool GUIOverlay::IsInitialized() 
{
  return CEGUI::WindowManager::getSingletonPtr() != NULL;
}
  
/// Load a CEGUI layout file
CEGUI::Window *GUIOverlay::LoadLayout( const std::string &_filename )
{
  CEGUI::Window *window = NULL;
  CEGUI::Window *rootWindow = NULL;

  CEGUI::WindowManager *windowManager = CEGUI::WindowManager::getSingletonPtr();
  if (!windowManager)
  {
    gzerr << "Attempting to create a GUI overlay window before load\n";
    return window;
  }

  rootWindow = windowManager->getWindow("root");
  if (rootWindow)
  {
    window = windowManager->loadWindowLayout( _filename );
    rootWindow->addChildWindow( window );
  }
  else
  {
    gzerr << "Attempting to create a GUI overlay window before load\n";
  }

  return window;
}

void GUIOverlay::OnConfig( const boost::shared_ptr<msgs::GUIOverlayConfig const> &_msg)
{
  this->configMsg = _msg;
}

void GUIOverlay::PreRender()
{
#ifdef HAVE_CEGUI
  if (this->configMsg)
  {
    this->LoadLayout( this->configMsg->layout_filename() );
    this->configMsg.reset();
  }
#endif
}

