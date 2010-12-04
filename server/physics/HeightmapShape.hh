/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
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

#endif
