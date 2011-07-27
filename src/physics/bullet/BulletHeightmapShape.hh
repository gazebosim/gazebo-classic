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
/* Desc: Trimesh geometry
 * Author: Nate Keonig, Andrew Howard
 * Date: 8 May 2003
 * CVS: $Id: BulletHeightmapGeom.hh 7640 2009-05-13 02:06:08Z natepak $
 */

#ifndef BULLETHEIGHTMAPGEOM_HH
#define BULLETHEIGHTMAPGEOM_HH

//#include <btBulletDynamicsCommon.h>
//#include <btBulletCollisionCommon.h>
#include "math::Vector2.hh"
#include "BulletGeom.hh"

class btHeightfieldTerrainShape;

namespace gazebo
{
	namespace physics
{
  class OgreHeightmap;

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
  class BulletHeightmapGeom : public BulletGeom
  {
    /// \brief Constructor
    public: BulletHeightmapGeom(Link *body);

    /// \brief Destructor
    public: virtual ~BulletHeightmapGeom();

    /// \brief Update function 
    public: void Update();

    /// \brief Load the heightmap
    public: virtual void Load(common::XMLConfigNode *node);

    /// \brief Save child parameters
    public: void Save(std::string &prefix, std::ostream &stream);

    /// Create a lookup table of the terrain's height
    private: void FillHeightMap();

    private: math::Vector3 terrainSize;

    private: common::ParamT<std::string> *imageFilenameP;
    private: common::ParamT<std::string> *worldTextureP;
    private: common::ParamT<std::string> *detailTextureP;
    private: common::ParamT<math::Vector3> *sizeP;
    private: common::ParamT<math::Vector3> *offsetP;

    private: OgreHeightmap *ogreHeightmap;

    private: int width, height;
    private: btHeightfieldTerrainShape* heightFieldShape;
  };

  /// \}
}

}
}
#endif
