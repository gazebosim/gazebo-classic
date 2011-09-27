#include "CEGUI/CEGUI.h"
#include "CEGUI/RendererModules/Ogre/CEGUIOgreRenderer.h"
#include "GUIOverlay.hh"

using namespace gazebo;
using namespace rendering;

GUIOverlay::GUIOverlay()
{
}

GUIOverlay::~GUIOverlay()
{
  CEGUI::OgreRenderer::destroySystem();
}

void GUIOverlay::Init( Ogre::RenderTarget *_renderTarget )
{
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
  this->rootWindow = CEGUI::WindowManager::getSingleton().createWindow("DefaultWindow", "root");
  CEGUI::System::getSingleton().setGUISheet( this->rootWindow );

  /*CEGUI::FrameWindow *frameWindow = (CEGUI::FrameWindow*)CEGUI::WindowManager::getSingleton().createWindow("TaharezLook/FrameWindow", "testWindow");
  this->rootWindow->addChildWindow( frameWindow );

  frameWindow->setPosition( CEGUI::UVector2( CEGUI::UDim( 0.5, 0), 
                                             CEGUI::UDim(0.5, 0) ) );

  frameWindow->setSize( CEGUI::UVector2( CEGUI::UDim( 0.25, 0), CEGUI::UDim(0.25, 0) ) );
  frameWindow->setText( "Hello World\n" );
  */
}
