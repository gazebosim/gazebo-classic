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
/* Desc: Heightmap collision
 * Author: Nate Keonig
 * Date: 8 May 2003
 */

#ifndef __BULLETHEIGHTMAPGEOM_HH__
#define __BULLETHEIGHTMAPGEOM_HH__
#include <string>

#include "physics/HeightmapShape.hh"
#include "physics/bullet/BulletPhysics.hh"
#include "physics/Collision.hh"

class btHeightfieldTerrainShape;

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{
    /// \addtogroup gazebo_physics_bulet Bullet Physics
    /// \{
    /// \brief Height map collision
    class BulletHeightmapShape : public HeightmapShape
    {
      /// \brief Constructor
      public: BulletHeightmapShape(CollisionPtr _parent);

      /// \brief Destructor
      public: virtual ~BulletHeightmapShape();

      /// \brief Load the heightmap
      public: virtual void Init();

      /// \brief Get a height at a vertex
      public: virtual float GetHeight(int x, int y);

      private: btHeightfieldTerrainShape* heightFieldShape;
    };
    /// \}
    /// \}
  }
}
#endif
