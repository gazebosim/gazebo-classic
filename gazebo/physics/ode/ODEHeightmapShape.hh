/*
 * Copyright 2011 Nate Koenig
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
/* Desc: ODE Heightmap shape
 * Author: Nate Koenig
 * Date: 12 Nov 2009
 */

#ifndef ODEHEIGHTMAPSHAPE_HH
#define ODEHEIGHTMAPSHAPE_HH

#include <vector>

#include "physics/HeightmapShape.hh"
#include "physics/ode/ODEPhysics.hh"
#include "physics/Collision.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_ode ODE Physics
    /// \{

    /// \brief ODE Height map collision
    class ODEHeightmapShape : public HeightmapShape
    {
      /// \brief Constructor
      public: ODEHeightmapShape(CollisionPtr _parent);

      /// \brief Destructor
      public: virtual ~ODEHeightmapShape();

      /// \brief Load the heightmap
      public: virtual void Init();

      /// \brief Called by ODE to get the height at a vertex
      private: static dReal GetHeightCallback(void *data, int x, int y);

      private: dHeightfieldDataID odeData;
    };

    /// \}
  }
}
#endif
