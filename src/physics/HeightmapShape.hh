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
/* Desc: Heightmap shape
 * Author: Nate Keonig, Andrew Howard
 * Date: 8 May 2003
 * CVS: $Id$
 */

#ifndef HEIGHTMAPSHAPE_HH
#define HEIGHTMAPSHAPE_HH

#include "Vector2.hh"
#include "Image.hh"
#include "Geom.hh"


namespace gazebo
{
	namespace physics
{
// NATY:  class OgreHeightmap;

  /// \addtogroup gazebo_physics_geom
  /// \{
  /** \defgroup gazebo_heightmap_geom Height map geom
      \brief Height map geom

    \par Attributes
    The following attributes are supported.

    - image (string)
      - Greyscale image to use as the height map
      - Default: (empty)

    - worldTexture (string)
      - Material to use on distant portions of the heightmap, relative to the camera's pose
      - Default: (empty)

    - detailTexture (string)
      - Material to use on nearby portions of the heightmap, relative to the camera's pose
      - Default: (empty)

    - size (float tuple)
      - Size of the height map
      - Default: 0 0 0

    \par Example
    \verbatim
      <geom:heightmap name="terrain_geom">
        <image>terrain.png</image>
        <worldTexture>terrain_texture.jpg</worldTexture>
        <detailTexture>terrain_detail.jpg</detailTexture>
        <size>1000 1000 10.0</size>
      </geom:heightmap>
    \endverbatim
    */
  /// \}
  /// \addtogroup gazebo_heightmap_geom 
  /// \{


  /// \brief Height map geom
  class HeightmapShape : public Shape
  {
    /// \brief Constructor
    public: HeightmapShape(Geom *parent);

    /// \brief Destructor
    public: virtual ~HeightmapShape();

    /// \brief Update function 
    public: void Update();

    /// \brief Load the heightmap
    protected: virtual void Load(XMLConfigNode *node);

    /// \brief Save child parameters
    protected: void Save(std::string &prefix, std::ostream &stream);

    protected: Vector3 terrainSize;

    protected: std::vector<double> heights;

    protected: Image img;
    protected: ParamT<std::string> *imageFilenameP;
    protected: ParamT<std::string> *worldTextureP;
    protected: ParamT<std::string> *detailTextureP;
    protected: ParamT<Vector3> *sizeP;
    protected: ParamT<Vector3> *offsetP;

    // NATY: protected: OgreHeightmap *ogreHeightmap;
  };

  /// \}
}

}
#endif
