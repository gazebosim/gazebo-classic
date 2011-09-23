#include "GuiOverlay.hh"

using namespace gazebo;
using namespace rendering;

GuiOverlay::GuiOverlay()
{
}

GuiOverlay::~GuiOverlay()
{
}

void GuiOverlay::Init( Ogre::RenderWindow *_renderWindow )
{
  this->guiRenderer = new CEGUI::OgreCEGUIRenderer( _renderWindow );
  this->guiSystem = new CEGUI::System( this->guiRenderer );
}

