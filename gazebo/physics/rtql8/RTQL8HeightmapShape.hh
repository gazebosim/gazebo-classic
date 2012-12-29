/*
 * Copyright 2012 Open Source Robotics Foundation
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

#ifndef _RTQL8HEIGHTMAPSHAPE_HH_
#define _RTQL8HEIGHTMAPSHAPE_HH_

#include <vector>

#include "gazebo/physics/HeightmapShape.hh"
#include "gazebo/physics/rtql8/RTQL8Physics.hh"
#include "gazebo/physics/Collision.hh"

namespace gazebo
{
  namespace physics
  {
    /// \brief RTQL8 Height map collision.
    class RTQL8HeightmapShape : public HeightmapShape
    {
      /// \brief Constructor.
      /// \param[in] _parent Collision parent.
      public: RTQL8HeightmapShape(CollisionPtr _parent);

      /// \brief Destructor
      public: virtual ~RTQL8HeightmapShape();

      // Documentation inerited.
      public: virtual void Init();

      /// \brief Called by ODE to get the height at a vertex.
      /// \param[in] _data Pointer to the heightmap data.
      /// \param[in] _x X location.
      /// \param[in] _y Y location.
      //private: static dReal GetHeightCallback(void *_data, int _x, int _y);

      /// \brief The heightmap data.
      //private: dHeightfieldDataID odeData;
    };
  }
}
#endif
