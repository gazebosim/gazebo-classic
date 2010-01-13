#ifndef ASSIMPLOADER_HH
#define ASSIMPLOADER_HH

#include <string>

#include <assimp.hpp>
#include <aiScene.h>
#include <aiPostProcess.h>

#include "MeshLoader.hh"

namespace gazebo
{
  /// \brief Wrapper around the Assimp asset loader
  class AssimpLoader : public MeshLoader
  {
    /// \brief Constructor
    public: AssimpLoader();

    /// \brief Destructor
    public: virtual ~AssimpLoader();
  
    /// \brief Load a mesh
    public: virtual Mesh *Load( const std::string &filename );
  
    /// \brief Construct the mesh
    private: void BuildMesh(aiNode *node, Mesh *mesh);
  
    private: Assimp::Importer importer;
  };
}

#endif
