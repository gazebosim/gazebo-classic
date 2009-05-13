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
/* Desc: Heightmap geometry
 * Author: Nate Keonig
 * Date: 12 May 2009
 * SVN: $Id:$
 */

#ifndef OGREHEIGHTMAP_HH
#define OGREHEIGHTMAP_HH

#include <Ogre.h>

#include "Vector2.hh"
#include "Vector3.hh"

namespace gazebo
{

  /// \brief Height map geom
  class OgreHeightmap : public Ogre::RaySceneQueryListener
  {
    /// \brief Constructor
    public: OgreHeightmap();

    /// \brief Destructor
    public: virtual ~OgreHeightmap();

    /// \brief Get the height of the heightmap as a specific coordinate
    public: float GetHeightAt(const Vector2<float> &pos);

    /// \brief Overloaded Ogre function for Ray Scene Queries
    public: virtual bool queryResult(Ogre::MovableObject *obj, Ogre::Real dist);

    /// \brief Overloaded Ogre function for Ray Scene Queries
    public: virtual bool queryResult(Ogre::SceneQuery::WorldFragment *frag, Ogre::Real dist);

    /// \brief Load the heightmap
    public: virtual void Load( std::string imageFilename, 
                                  std::string worldTexture, 
                                  std::string detialTexture,
                                  Vector3 terrainSize);

    private: Vector3 terrainSize;

    private: Ogre::Ray ray;
    private: Ogre::RaySceneQuery *rayQuery;

    private: double distToTerrain;
  };
}

#endif
