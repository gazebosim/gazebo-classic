#ifndef OGRETEXTRENDERER_HH
#define OGRETEXTRENDERER_HH

#include <Ogre.h>

namespace gazebo
{

  class OgreTextRenderer
  {
    /// \brief Constructor
    private: OgreTextRenderer();

    /// \brief Destructor
    private: ~OgreTextRenderer();

    /// \brief Get a pointer to the text renderer
    public: static OgreTextRenderer *Instance();

    /// \brief Add a text box
    public: void AddTextBox( const std::string& id,
                const std::string& text,
                Ogre::Real x, 
                Ogre::Real y,
                Ogre::Real width, 
                Ogre::Real height,
                const Ogre::ColourValue& color = 
                Ogre::ColourValue(1.0, 1.0, 1.0, 1.0));

    /// \brief Remove a text box
    public: void RemoveTextBox(const std::string& id);

    /// \brief Set text 
    public: void SetText(const std::string& id, const std::string& Text);

    private: Ogre::OverlayManager *overlayMgr;
    private: Ogre::Overlay *overlay;
    private: Ogre::OverlayContainer *panel;

    private: static OgreTextRenderer *myself;
  };

}
#endif
