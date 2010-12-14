#ifndef SELECTION_OBJ
#define SELECTION_OBJ

#include "Vector3.hh"


namespace gazebo
{
  class Visual;
  class Scene;

  class SelectionObj
  {
    /// \brief Constructor
    public: SelectionObj();

    /// \brief Destructor
    public: virtual ~SelectionObj();

    public: void Load();

    /// \brief Set the position of the node
    public: void Attach( Visual *visual );

    private: Visual *node;
    private: Scene *scene;
  };
}

#endif
