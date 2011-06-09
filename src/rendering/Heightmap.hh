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
 * SVN: $Id:$
 */

#ifndef HEIGHTMAP_HH
#define HEIGHTMAP_HH

// TODO: remove this line
#include "rendering/ogre.h"

#include "common/Vector3.hh"
#include "common/Vector2d.hh"

namespace gazebo
{
	namespace rendering
  {
    class Scene;
  
    /// \brief Height map geom
    class Heightmap : public Ogre::RaySceneQueryListener
    {
      /// \brief Constructor
      public: Heightmap(Scene *scene);
  
      /// \brief Destructor
      public: virtual ~Heightmap();
  
      /// \brief Get the height of the heightmap as a specific coordinate
      public: float GetHeightAt(const common::Vector2d &pos);
  
      /// \brief Overloaded Ogre function for Ray Scene Queries
      public: virtual bool queryResult(Ogre::MovableObject *obj, Ogre::Real dist);
  
      /// \brief Overloaded Ogre function for Ray Scene Queries
      public: virtual bool queryResult(Ogre::SceneQuery::WorldFragment *frag, Ogre::Real dist);
  
      /// \brief Load the heightmap
      public: virtual void Load( std::string imageFilename, 
                                    std::string worldTexture, 
                                    std::string detialTexture,
                                    common::Vector3 terrainSize);
  
      private: common::Vector3 terrainSize;
  
      private: Ogre::Ray ray;
      private: Ogre::RaySceneQuery *rayQuery;
  
      private: double distToTerrain;
      private: Scene *scene;
    };
  }

}
#endif
