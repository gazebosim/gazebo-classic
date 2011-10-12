#include "rendering/GUIOverlay.hh"
#include "common/Console.hh"

#ifdef HAVE_CEGUI
#include "CEGUI/CEGUI.h"
#include "CEGUI/RendererModules/Ogre/CEGUIOgreRenderer.h"
#endif

#include "common/SystemPaths.hh"
#include "rendering/ogre.h"
#include "msgs/msgs.h"
#include "transport/Transport.hh"
#include "transport/Node.hh"
#include "math/Vector2d.hh"
#include "rendering/Camera.hh"

using namespace gazebo;
using namespace rendering;

GUIOverlay::GUIOverlay()
{
  this->connections.push_back( 
      event::Events::ConnectPreRender( 
        boost::bind(&GUIOverlay::PreRender, this) ) );
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
  CEGUI::System::getSingletonPtr();

  std::string logPath = common::SystemPaths::Instance()->GetLogPath();
  logPath += "/cegui.log";

  this->guiRenderer = &CEGUI::OgreRenderer::create(*_renderTarget);
  CEGUI::OgreResourceProvider &ip = CEGUI::OgreRenderer::createOgreResourceProvider();
  CEGUI::OgreImageCodec &ic = CEGUI::OgreRenderer::createOgreImageCodec();

  CEGUI::System::create(*((CEGUI::Renderer*)this->guiRenderer), 
      (CEGUI::ResourceProvider*)(&ip), 
      static_cast<CEGUI::XMLParser*>(0), 
      (CEGUI::ImageCodec*)(&ic),
      NULL, "", logPath);

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
#ifdef HAVE_CEGUI
  return CEGUI::WindowManager::getSingletonPtr() != NULL;
#else
  return false;
#endif
}
  
/// Load a CEGUI layout file
void GUIOverlay::LoadLayout( const std::string &_filename )
{
  this->layoutFilename = _filename;
}

void GUIOverlay::PreRender()
{
#ifdef HAVE_CEGUI
  if (this->IsInitialized() && !this->layoutFilename.empty())
  {
    this->LoadLayoutImpl( this->layoutFilename );
    this->layoutFilename.clear();
  }
#endif
}

/// Load a CEGUI layout file
#ifdef HAVE_CEGUI
CEGUI::Window *GUIOverlay::LoadLayoutImpl( const std::string &_filename )
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
#endif

void GUIOverlay::Resize( unsigned int _width, unsigned int _height )
{
#ifdef HAVE_CEGUI
  this->guiRenderer->setDisplaySize( CEGUI::Size(_width, _height) );

  CEGUI::WindowManager *windowManager = CEGUI::WindowManager::getSingletonPtr();

  CEGUI::Window *rootWindow = windowManager->getWindow("root");
  rootWindow->setArea( CEGUI::UDim(0,0), CEGUI::UDim(0,0), CEGUI::UDim(1,0), CEGUI::UDim(1,0));
#endif
}

bool GUIOverlay::AttachCameraToImage(CameraPtr &_camera, const std::string &_windowName)
{
#ifdef HAVE_CEGUI
  CEGUI::Window *window = NULL;
  CEGUI::WindowManager *windowManager = CEGUI::WindowManager::getSingletonPtr();

  if (!windowManager)
  {
    gzerr << "CEGUI system not initialized\n";
    return false;
  }

  window = windowManager->getWindow(_windowName);
  if (!window)
  {
    gzerr << "Unable to find window[" << _windowName << "]\n";
    return false;
  }

  if (!_camera->GetRenderTexture())
  {
    gzerr << "Camera does not have a render texture\n";
    return false;
  }

#ifdef HAVE_CEGUI
  Ogre::TexturePtr texPtr(_camera->GetRenderTexture());
  CEGUI::Texture &guiTex = this->guiRenderer->createTexture(
      texPtr );

  CEGUI::Imageset &imageSet = CEGUI::ImagesetManager::getSingleton().create("RTTImageset", guiTex);

  imageSet.defineImage("RTTImage", CEGUI::Point(0.0f, 0.0f),
      CEGUI::Size(guiTex.getSize().d_width, 
                  guiTex.getSize().d_height),
      CEGUI::Point(0.0f, 0.0f));

  window->setProperty("Image", CEGUI::PropertyHelper::imageToString(&imageSet.getImage("RTTImage")));
  return true;

#endif
#endif
  return false;
}
