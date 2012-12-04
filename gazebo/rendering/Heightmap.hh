/*
 * Copyright 2012 Open Source Robotics Foundation
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
/* Desc: Heightmap geometry
 * Author: Nate Koenig
 * Date: 12 May 2009
 */

#ifndef _HEIGHTMAP_HH_
#define _HEIGHTMAP_HH_

#include <string>
#include <vector>

#include "common/Image.hh"
#include "math/Vector3.hh"
#include "math/Vector2d.hh"
#include "rendering/Scene.hh"

namespace Ogre
{
  class TerrainGlobalOptions;
  class TerrainGroup;
  class Terrain;
}

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{

    /// \class Heightmap Heightmap.hh rendering/rendering.hh
    /// \brief Rendering a terrain using heightmap information
    class Heightmap
    {
      /// \brief Constructor
      /// \param[in] _scene Pointer to the scene that will contain the heightmap
      public: Heightmap(ScenePtr _scene);

      /// \brief Destructor
      public: virtual ~Heightmap();

      /// \brief Load the heightmap
      public: void Load();

      /// \brief Load the heightmap from a visual message
      /// \param[in] _msg The visual message containing heightmap info
      public: void LoadFromMsg(ConstVisualPtr &_msg);

      /// \brief Get the height at a location
      /// \param[in] _x X location
      /// \param[in] _y Y location
      /// \param[in] _z Z location
      /// \return The height at the specified location
      public: double GetHeight(double _x, double _y, double _z = 1000);

      /// \brief Initialize all the blend material maps.
      /// \param[in] _terrain The terrain to initialize the blend maps.
      private: bool InitBlendMaps(Ogre::Terrain *_terrain);

      /// \brief Configure the terrain default values.
      private: void ConfigureTerrainDefaults();

      /// \brief Define a section of the terrain.
      /// \param[in] _x X coordinate of the terrain.
      /// \param[in] _y Y coordinate of the terrain.
      private: void DefineTerrain(int _x, int _y);

      /// \brief The scene.
      private: ScenePtr scene;

      /// \brief Image used to generate the heightmap.
      private: common::Image heightImage;

      /// \brief Size of the terrain.
      private: math::Vector3 terrainSize;

      /// \brief Size of the image.
      private: unsigned int imageSize;

      /// \brief Max pixel value.
      private: double maxPixel;

      /// \brief Origin of the terrain.
      private: math::Vector3 terrainOrigin;

      /// \brief Global options.
      private: Ogre::TerrainGlobalOptions *terrainGlobals;

      /// \brief Group of terrains.
      private: Ogre::TerrainGroup *terrainGroup;

      /// \brief True if the terrain was imported.
      private: bool terrainsImported;

      /// \brief The diffuse textures.
      private: std::vector<std::string> diffuseTextures;

      /// \brief The normal textures.
      private: std::vector<std::string> normalTextures;

      /// \brief The size of the world sections.
      private: std::vector<double> worldSizes;

      /// \brief The material blending heights.
      private: std::vector<double> blendHeight;

      /// \brief Material blend fade distances.
      private: std::vector<double> blendFade;
    };
    /// \}
  }
}
#endif
