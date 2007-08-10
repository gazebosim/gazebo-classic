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
 * CVS: $Id: TrimeshGeom.hh,v 1.1.2.1 2006/12/16 22:41:16 natepak Exp $
 */

#ifndef HEIGHTMAPGEOM_HH
#define HEIGHTMAPGEOM_HH

#include <Ogre.h>

#include "Geom.hh"

namespace gazebo
{
  /// \addtogroup gazebo_physics_geom
  /// \brief Height Map geom
  /// \{
  /// \defgroup gazebo_heightmap_geom Height map geom
  /// \brief Height map geom
  /// \{

  /// \brief Height map geom
  class HeightmapGeom : public Geom, public Ogre::RaySceneQueryListener
  {
    /// \brief Constructor
    public: HeightmapGeom(Body *body, const std::string &imageFilename, const std::string &worldTexture, const std::string &detailTexture, const Vector3 &size, const Vector3 &offset);

    /// \brief Destructor
    public: virtual ~HeightmapGeom();

    /// \brief Update function 
    public: void Update();

    /// \brief Get the height of the heightmap as a specific coordinate
    public: double GetHeightAt(const Vector2 &pos);

    /// \brief Overloaded Ogre function for Ray Scene Queries
    public: virtual bool queryResult(Ogre::MovableObject *obj, Ogre::Real dist);

    /// \brief Overloaded Ogre function for Ray Scene Queries
    public: virtual bool queryResult(Ogre::SceneQuery::WorldFragment *frag, Ogre::Real dist);

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
  };

  /// \}
  /// \}
}

#endif
