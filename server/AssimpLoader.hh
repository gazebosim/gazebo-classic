#ifndef ASSIMPLOADER_HH
#define ASSIMPLOADER_HH

#include <string>

#include <assimp/assimp.hpp>
#include <assimp/aiScene.h>
#include <assimp/aiPostProcess.h>

#include "MeshLoader.hh"

namespace gazebo
{
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
