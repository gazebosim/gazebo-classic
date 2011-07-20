/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef ASSIMPLOADER_HH
#define ASSIMPLOADER_HH

#include <string>

#include <assimp/assimp.hpp>
#include <assimp/aiScene.h>
#include <assimp/aiPostProcess.h>

#include "common/MeshLoader.hh"

namespace gazebo
{
  /// \ingroup gazebo_common
  /// \brief Common namespace
  namespace common
  {
    /// \addtogroup gazebo_common Common 
    /// \brief A set of common class, functions, and definitions
    /// \{
    
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
    /// \}
  }
}

#endif
