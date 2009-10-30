#ifndef LOADER_HH
#define LOADER_HH

#include <string>

namespace gazebo
{
  class Mesh;

  class MeshLoader
  {
    /// \brief Constructor
    public: MeshLoader();

    /// \brief Destructor
    public: virtual ~MeshLoader();

    /// \brief Load a 3D mesh
    public: virtual Mesh *Load(const std::string &filename) = 0;
  };
}

#endif
