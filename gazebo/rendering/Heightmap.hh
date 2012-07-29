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
/* Desc: Heightmap geometry
 * Author: Nate Keonig
 * Date: 12 May 2009
 */

#ifndef HEIGHTMAP_HH
#define HEIGHTMAP_HH
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
    /// \brief Height map geom
    class Heightmap
    {
      /// \brief Constructor
      public: Heightmap(ScenePtr scene);

      /// \brief Destructor
      public: virtual ~Heightmap();

      /// \brief Load the heightmap
      public: void Load();

      /// \brief Load the heightmap from a visual message
      public: void LoadFromMsg(ConstVisualPtr &_msg);

      /// \brief Get the height at a location
      public: double GetHeight(double x, double y);

      private: bool InitBlendMaps(Ogre::Terrain *_terrain);
      private: void ConfigureTerrainDefaults();
      private: void DefineTerrain(int x, int y);

      private: ScenePtr scene;
      private: common::Image heightImage;
      private: math::Vector3 terrainSize;
      private: unsigned int imageSize;
      private: double maxPixel;
      private: math::Vector3 terrainOrigin;

      private: Ogre::TerrainGlobalOptions *terrainGlobals;
      private: Ogre::TerrainGroup *terrainGroup;
      private: bool terrainsImported;

      private: std::vector<std::string> diffuseTextures;
      private: std::vector<std::string> normalTextures;
      private: std::vector<double> worldSizes;

      private: std::vector<double> blendHeight;
      private: std::vector<double> blendFade;
    };
    /// \}
  }
}
#endif
