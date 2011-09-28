#include "rendering/GUIOverlay.hh"
#include "common/Console.hh"

#ifdef HAVE_CEGUI
#include "CEGUI/CEGUI.h"
#include "CEGUI/RendererModules/Ogre/CEGUIOgreRenderer.h"
#endif

#include "math/Vector2d.hh"

using namespace gazebo;
using namespace rendering;

GUIOverlay::GUIOverlay()
{
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
  CEGUI::FontManager::getSingleton().create("DejaVuSans-10.font");
  //CEGUI::System::getSingleton().setDefaultMouseCursor("TaharezLook", "MouseArrow");


  // clearing this queue actually make sure it's created
  this->guiRenderer->getDefaultRenderingRoot().clearGeometry(CEGUI::RQ_OVERLAY);

  // Create a root window, and set it as the root window
  this->windows["root"] = CEGUI::WindowManager::getSingleton().createWindow("DefaultWindow", "root");
  CEGUI::System::getSingleton().setGUISheet( this->windows["root"] );

  //this->CreateWindow("TaharezLook/Button", "VerbButton", "root",
      //math::Vector2d(0.2, 0.2), math::Vector2d(0.2, 0.2), "Verb");
  //this->SubscribeEvent( "VerbButton", &GUIOverlay::OnButton, this);
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
  std::map<std::string, CEGUI::Window*>::iterator iter;
  iter = this->windows.find(_parent);
  if (iter == this->windows.end())
  {
    gzerr << "Unable to find parent window[" << _parent << "]\n";
    return;
  }

  CEGUI::Window *window = CEGUI::WindowManager::getSingleton().createWindow(_type, _name);

  iter->second->addChildWindow( window );

  window->setPosition( CEGUI::UVector2( CEGUI::UDim(_position.x, 0), 
                                             CEGUI::UDim(_position.y, 0) ) );

  window->setSize( CEGUI::UVector2( CEGUI::UDim(_size.x, 0), 
                                         CEGUI::UDim(_size.y, 0) ) );
  window->setText( _text );

  this->windows[_name] = window;
#endif
}

bool GUIOverlay::HandleMouseEvent( const common::MouseEvent &_evt)
{
  bool result = false;
#ifdef HAVE_CEGUI
  CEGUI::System *system = CEGUI::System::getSingletonPtr();
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
