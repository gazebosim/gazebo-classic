#ifndef LIGHT_HH
#define LIGHT_HH

#include <string>
#include <iostream>

#include "Entity.hh"

namespace Ogre
{
  class Light;
}

namespace gazebo
{
  class OgreVisual;
  class XMLConfigNode;
  class OgreDynamicLines;

  /// \brief Wrapper around an ogre light source
  class Light : public Entity
  {
    /// \brief Constructor
    public: Light(Entity *parent);

    /// \brief Destructor
    public: virtual ~Light();

    /// \brief Load the light
    public: void Load(XMLConfigNode *node);

    /// \brief Save a light
    public: void Save(const std::string &prefix, std::ostream &stream);

    /// \brief Set whether this entity has been selected by the user through  
    ///        the gui
    public: virtual bool SetSelected( bool s );

    /// \private Helper node to create a visual representation of the light
    private: void CreateVisual();

    /// The OGRE light source
    private: Ogre::Light *light;

    /// Parent of the light
    private: OgreVisual *visual;
    private: OgreDynamicLines *line;

    private: static unsigned int lightCounter;
  };
}
#endif
