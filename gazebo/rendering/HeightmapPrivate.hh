/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#ifndef _GAZEBO_RENDERING_HEIGHTMAP_PRIVATE_HH_
#define _GAZEBO_RENDERING_HEIGHTMAP_PRIVATE_HH_

#include <string>
#include <vector>
#include <boost/filesystem.hpp>

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Vector2d.hh"
#include "gazebo/rendering/Scene.hh"

namespace Ogre
{
  class TerrainGlobalOptions;
  class TerrainGroup;
}

namespace gazebo
{
  namespace rendering
  {
    /// \internal
    /// \brief Private data for the Heightmap class
    class HeightmapPrivate
    {
      /// \brief Number of pieces in which a terrain is subdivided for paging.
      public: static const unsigned int numTerrainSubdivisions;

      /// \brief The terrain pages are loaded if the distance from the camera is
      /// within the loadRadius. See Ogre::TerrainPaging::createWorldSection().
      /// LoadRadiusFactor is a multiplier applied to the terrain size to create
      /// a load radius that depends on the terrain size.
      public: static const double loadRadiusFactor;

      /// \brief The terrain pages are held in memory but not loaded if they
      /// are not ready when the camera is within holdRadius distance. See
      /// Ogre::TerrainPaging::createWorldSection(). HoldRadiusFactor is a
      /// multiplier applied to the terrain size to create a hold radius that
      /// depends on the terrain size.
      public: static const double holdRadiusFactor;

      /// \brief Hash file name that should be present for every terrain file
      /// loaded using paging.
      public: static const boost::filesystem::path hashFilename;

      /// \brief Name of the top level directory where all the paging info is
      /// stored
      public: static const boost::filesystem::path pagingDirname;

      /// \brief When the terrain paging is enabled, the terrain information
      /// for every piece of terrain is stored in disk. This is the path of
      /// the top level directory where these files are located.
      public: boost::filesystem::path gzPagingDir;

      /// \brief The scene.
      public: ScenePtr scene;

      /// \brief Size of the terrain.
      public: math::Vector3 terrainSize;

      /// \brief Size of the heightmap data.
      public: unsigned int dataSize;

      /// \brief Origin of the terrain.
      public: math::Vector3 terrainOrigin;

      /// \brief Global options.
      public: Ogre::TerrainGlobalOptions *terrainGlobals;

      /// \brief Group of terrains.
      public: Ogre::TerrainGroup *terrainGroup;

      /// \brief True if the terrain was imported.
      public: bool terrainsImported;

      /// \brief The diffuse textures.
      public: std::vector<std::string> diffuseTextures;

      /// \brief The normal textures.
      public: std::vector<std::string> normalTextures;

      /// \brief The size of the world sections.
      public: std::vector<double> worldSizes;

      /// \brief The material blending heights.
      public: std::vector<double> blendHeight;

      /// \brief Material blend fade distances.
      public: std::vector<double> blendFade;

      /// \brief The raw height values received from physics.
      public: std::vector<float> heights;

      /// \brief Pointer to the terrain material generator.
      public: GzTerrainMatGen *gzMatGen;

      /// \brief A page provider is needed to use the paging system.
      public: DummyPageProvider dummyPageProvider;

      /// \brief Central registration point for extension classes,
      /// such as the PageStrategy, PageContentFactory.
      public: Ogre::PageManager *pageManager;

      /// \brief Type of paging applied
      public: Ogre::TerrainPaging *terrainPaging;

      /// \brief Collection of world content
      public: Ogre::PagedWorld *world;

      /// \brief Collection of terrains. Every terrain might be paged.
      public: std::vector<std::vector<float> > subTerrains;

      /// \brief Used to iterate over all the terrains
      public: int terrainIdx;

      /// \brief Flag that enables/disables the terrain paging
      public: bool useTerrainPaging;

      /// \brief True if the terrain's hash does not match the image's hash
      public: bool terrainHashChanged;
    };
  }
}
#endif
