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
/* Desc: Trimesh geometry
 * Author: Nate Keonig, Andrew Howard
 * Date: 8 May 2003
 * CVS: $Id$
 */

#ifndef HEIGHTMAPGEOM_HH
#define HEIGHTMAPGEOM_HH

#include <Ogre.h>

#include "Vector2.hh"
#include "Geom.hh"

namespace gazebo
{
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
  class HeightmapGeom : public Geom, public Ogre::RaySceneQueryListener
  {
    /// \brief Constructor
    public: HeightmapGeom(Body *body);

    /// \brief Destructor
    public: virtual ~HeightmapGeom();

    /// \brief Update function 
    public: void UpdateChild();

    /// \brief Get the height of the heightmap as a specific coordinate
    public: float GetHeightAt(const Vector2<float> &pos);

    /// \brief Overloaded Ogre function for Ray Scene Queries
    public: virtual bool queryResult(Ogre::MovableObject *obj, Ogre::Real dist);

    /// \brief Overloaded Ogre function for Ray Scene Queries
    public: virtual bool queryResult(Ogre::SceneQuery::WorldFragment *frag, Ogre::Real dist);

    /// \brief Load the heightmap
    protected: virtual void LoadChild(XMLConfigNode *node);

    /// \brief Save child parameters
    protected: void SaveChild(std::string &prefix, std::ostream &stream);

    /// Create a lookup table of the terrain's height
    private: void FillHeightMap();

    /// \brief Called by ODE to get the height at a vertex
    private: static dReal GetHeightCallback(void *data, int x, int y);

    private: dHeightfieldDataID odeData;

    private: Vector3 terrainSize;
    private: Vector3 terrainScale;

    private: unsigned int odeVertSize;
    private: Vector3 odeScale;

    private: unsigned int terrainVertSize;
    private: std::string terrainImage;

    private: Ogre::Ray ray;
    private: Ogre::RaySceneQuery *rayQuery;

    private: double distToTerrain;

    private: std::vector<double> heights;

    private: ParamT<std::string> *imageFilenameP;
    private: ParamT<std::string> *worldTextureP;
    private: ParamT<std::string> *detailTextureP;
    private: ParamT<Vector3> *sizeP;
    private: ParamT<Vector3> *offsetP;

  };

  /// \}
}

#endif
