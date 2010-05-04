#ifndef SELECTION_OBJ
#define SELECTION_OBJ

#include "Vector3.hh"

namespace Ogre
{
  class SceneNode;
}

namespace gazebo
{
  class SelectionObj
  {
    /// \brief Constructor
    public: SelectionObj();

    /// \brief Destructor
    public: virtual ~SelectionObj();

    public: void Load();

    /// \brief Set the position of the node
    public: void Attach( Ogre::SceneNode *node );

    private: Ogre::SceneNode *node;
  };
}

#endif
