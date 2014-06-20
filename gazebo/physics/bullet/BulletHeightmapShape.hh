/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
 * Author: Nate Koenig
 * Date: 8 May 2003
 */

#ifndef _BULLETHEIGHTMAPGEOM_HH_
#define _BULLETHEIGHTMAPGEOM_HH_
#include <string>

#include "gazebo/physics/HeightmapShape.hh"
#include "gazebo/physics/bullet/BulletPhysics.hh"
#include "gazebo/physics/Collision.hh"

class btHeightfieldTerrainShape;

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_bullet Bullet Physics
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

      private: btHeightfieldTerrainShape* heightFieldShape;
    };
    /// \}
  }
}
#endif
