#include "OgreTextRenderer.hh"

using namespace gazebo;

OgreTextRenderer *OgreTextRenderer::myself = NULL;

////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor
OgreTextRenderer::OgreTextRenderer()
{
  this->overlayMgr = Ogre::OverlayManager::getSingletonPtr();

  this->overlay = this->overlayMgr->create("default");
  this->panel = static_cast<Ogre::OverlayContainer*>(this->overlayMgr->createOverlayElement("Panel", "container1"));
  this->panel->setDimensions(1, 1);
  this->panel->setPosition(0, 0);

  this->overlay->add2D(this->panel);

  this->overlay->show();
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
OgreTextRenderer::~OgreTextRenderer()
{
}

////////////////////////////////////////////////////////////////////////////////
// Get a pointer to the text renderer
OgreTextRenderer *OgreTextRenderer::Instance()
{
  if (!myself)
    myself = new OgreTextRenderer;

  return myself;
}

////////////////////////////////////////////////////////////////////////////////
/// Add a text box
void OgreTextRenderer::AddTextBox( const std::string& id,
                                   const std::string& text,
                                   Ogre::Real x, Ogre::Real y,
                                   Ogre::Real width, Ogre::Real height,
                                   const Ogre::ColourValue& color)
{
  Ogre::OverlayElement* textBox = this->overlayMgr->createOverlayElement("TextArea", id);
  textBox->setDimensions(width, height);
  textBox->setMetricsMode(Ogre::GMM_PIXELS);
  textBox->setPosition(x, y);
  textBox->setWidth(width);
  textBox->setHeight(height);
  textBox->setParameter("font_name", "Arial");
  textBox->setParameter("char_height", "16");
  textBox->setColour(color);

  textBox->setCaption(text);

  this->panel->addChild(textBox);
}

////////////////////////////////////////////////////////////////////////////////
/// Remove a text box
void OgreTextRenderer::RemoveTextBox(const std::string& ID)
{
  this->panel->removeChild(ID);
  this->overlayMgr->destroyOverlayElement(ID);
}

////////////////////////////////////////////////////////////////////////////////
/// Set text 
void OgreTextRenderer::SetText(const std::string& ID, const std::string& Text)
{
  Ogre::OverlayElement* textBox = this->overlayMgr->getOverlayElement(ID);
  textBox->setCaption(Text);
}
