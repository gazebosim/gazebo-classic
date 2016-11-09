/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_RENDERING_HEIGHTMAP_HH_
#define GAZEBO_RENDERING_HEIGHTMAP_HH_

#include <vector>
#include <string>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Vector2.hh>

#include "gazebo/msgs/MessageTypes.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/util/system.hh"

namespace boost
{
  namespace filesystem
  {
    class path;
  }
}

namespace Ogre
{
  class TerrainGroup;
  class Terrain;
}

namespace gazebo
{
  namespace math
  {
    class Vector2i;
  }

  namespace common
  {
    class Image;
  }

  namespace rendering
  {
    // Forward declare private data.
    class HeightmapPrivate;

    /// \addtogroup gazebo_rendering
    /// \{

    /// \class Heightmap Heightmap.hh rendering/rendering.hh
    /// \brief Rendering a terrain using heightmap information
    class GZ_RENDERING_VISIBLE Heightmap
    {
      /// \brief Constructor
      /// \param[in] _scene Pointer to the scene that will contain the heightmap
      public: explicit Heightmap(ScenePtr _scene);

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
      public: double Height(const double _x, const double _y,
          const double _z = 1000) const;

      /// \brief Flatten the terrain based on a mouse press.
      /// \param[in] _camera Camera associated with the mouse press.
      /// \param[in] _mousePos Position of the mouse in viewport
      /// coordinates.
      /// \param[in] _outsideRadius Controls the radius of effect.
      /// \param[in] _insideRadius Controls the size of the radius with the
      /// maximum effect (value between 0 and 1).
      /// \param[in] _weight Controls modification magnitude.
      /// \return True if the terrain was modified
      public: bool Flatten(CameraPtr _camera,
                          const ignition::math::Vector2i &_mousePos,
                          const double _outsideRadius,
                          const double _insideRadius,
                          const double _weight = 0.1);

      /// \brief Smooth the terrain based on a mouse press.
      /// \param[in] _camera Camera associated with the mouse press.
      /// \param[in] _mousePos Position of the mouse in viewport
      /// coordinates.
      /// \param[in] _outsideRadius Controls the radius of effect.
      /// \param[in] _insideRadius Controls the size of the radius with the
      /// maximum effect (value between 0 and 1).
      /// \param[in] _weight Controls modification magnitude.
      /// \return True if the terrain was modified
      public: bool Smooth(CameraPtr _camera,
                          const ignition::math::Vector2i &_mousePos,
                          const double _outsideRadius,
                          const double _insideRadius,
                          const double _weight = 0.1);

      /// \brief Raise the terrain based on a mouse press.
      /// \param[in] _camera Camera associated with the mouse press.
      /// \param[in] _mousePos Position of the mouse in viewport
      /// coordinates.
      /// \param[in] _outsideRadius Controls the radius of effect.
      /// \param[in] _insideRadius Controls the size of the radius with the
      /// maximum effect (value between 0 and 1).
      /// \param[in] _weight Controls modification magnitude.
      /// \return True if the terrain was modified
      public: bool Raise(CameraPtr _camera,
                         const ignition::math::Vector2i &_mousePos,
                         const double _outsideRadius,
                         const double _insideRadius,
                         const double _weight = 0.1);

      /// \brief Lower the terrain based on a mouse press.
      /// \param[in] _camera Camera associated with the mouse press.
      /// \param[in] _mousePos Position of the mouse in viewport
      /// coordinates.
      /// \param[in] _outsideRadius Controls the radius of effect.
      /// \param[in] _insideRadius Controls the size of the radius with the
      /// maximum effect (value between 0 and 1).
      /// \param[in] _weight Controls modification magnitude.
      /// \return True if the terrain was modified
      public: bool Lower(CameraPtr _camera,
                         const ignition::math::Vector2i &_mousePos,
                         const double _outsideRadius,
                         const double _insideRadius,
                         const double _weight = 0.1);

      /// \brief Get the average height around a point.
      /// \param[in] _pos Position in world coordinates.
      /// \param[in] _brushSize Controls the radius of effect.
      public: double AvgHeight(const ignition::math::Vector3d &_pos,
          const double _brushSize) const;

      /// \brief Set the heightmap to render in wireframe mode.
      /// \param[in] _show True to render wireframe, false to render solid.
      public: void SetWireframe(const bool _show);

      /// \brief Get a pointer to the OGRE terrain group object.
      /// \return Pointer to the OGRE terrain.
      public: Ogre::TerrainGroup *OgreTerrain() const;

      /// \brief Get the heightmap as an image
      /// \return An image that contains the terrain data.
      public: common::Image Image() const;

      /// \brief Calculate a mouse ray hit on the terrain.
      /// \param[in] _camera Camera associated with the mouse press.
      /// \param[in] _mousePos Position of the mouse in viewport
      /// coordinates.
      /// \return The result of the mouse ray hit.
      public: Ogre::TerrainGroup::RayResult MouseHit(CameraPtr _camera,
                  const ignition::math::Vector2i &_mousePos) const;

      /// \brief Split a terrain into subterrains
      /// \param[in] _heightmap Source vector of floats with the heights.
      /// \param[in] _n Number of subterrains.
      /// \param[out] _v Destination vector with the subterrains.
      public: void SplitHeights(const std::vector<float> &_heightmap,
                  const int _n, std::vector<std::vector<float> > &_v);

      /// \brief Get the number of subdivision the terrain will be split
      /// into.
      /// \return Number of terrain subdivisions
      public: unsigned int TerrainSubdivisionCount() const;

      /// \brief Set custom material for the terrain
      /// \param[in] _materialName Name of the material
      public: void SetMaterial(const std::string &_materialName);

      /// \brief Get the custom material name used for the terrain.
      /// \return Custom material name.
      public: std::string MaterialName() const;

      /// \brief Create terrain material generator. There are two types:
      /// custom material generator that support user material scripts,
      /// and a default material generator that uses our own glsl shader
      /// and supports PSSM shadows.
      private: void CreateMaterial();

      /// \brief Modify the height at a specific point.
      /// \param[in] _pos Position in world coordinates.
      /// \param[in] _outsideRadius Controls the radius of effect.
      /// \param[in] _insideRadius Controls the size of the radius with the
      /// maximum effect (value between 0 and 1).
      /// \param[in] _weight Controls modification magnitude.
      /// \param[in] _op Type of operation to perform.
      private: void ModifyTerrain(Ogre::Vector3 _pos,
                    const double _outsideRadius, const double _insideRadius,
                    const double _weight, const std::string &_op);

      /// \brief Initialize all the blend material maps.
      /// \param[in] _terrain The terrain to initialize the blend maps.
      private: bool InitBlendMaps(Ogre::Terrain *_terrain);

      /// \brief Configure the terrain default values.
      private: void ConfigureTerrainDefaults();

      /// \brief Define a section of the terrain.
      /// \param[in] _x X coordinate of the terrain.
      /// \param[in] _y Y coordinate of the terrain.
      private: void DefineTerrain(const int _x, const int _y);

      /// \brief Internal function used to setup shadows for the terrain.
      /// \param[in] _enabled True to enable shadows.
      private: void SetupShadows(const bool _enabled);

      /// \brief Update the hash of a terrain file. The hash will be written in
      /// a file called gzterrain.SHA1 . This method will be used when the
      /// paging is enabled and the terrain is loaded for the first time or if
      /// the heightmap's image has been modified.
      /// \param[in] _hash New hash value
      /// \param[in] _terrainDir Directory where the terrain hash and the
      /// terrain pages are stored. Ex: $TMP/gazebo-paging/heigthmap_bowl
      private: void UpdateTerrainHash(const std::string &_hash,
          const boost::filesystem::path &_terrainDir);

      /// \brief It checks if the terrain was previously loaded. In negative
      /// case, it splits the original terrain into pieces and creates a hash
      /// file.
      /// \param[in] _terrainDirPath Path to the directory containing the
      /// terrain pages and hash.
      /// \return True if the terrain requires to regenerate the terrain files.
      private: bool PrepareTerrainPaging(
        const boost::filesystem::path &_terrainDirPath);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<HeightmapPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
