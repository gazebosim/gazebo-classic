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
/* Desc: Plugin Heightmap shape
 * Author: Nate Koenig
 * Date: 12 Nov 2009
 */

#ifndef _PluginHEIGHTMAPSHAPE_HH_
#define _PluginHEIGHTMAPSHAPE_HH_

#include <vector>

#include "gazebo/physics/HeightmapShape.hh"
#include "gazebo/physics/plugin/PluginPhysics.hh"
#include "gazebo/physics/Collision.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \brief Plugin Height map collision.
    class GAZEBO_VISIBLE PluginHeightmapShape : public HeightmapShape
    {
      /// \brief Constructor.
      /// \param[in] _parent Collision parent.
      public: PluginHeightmapShape(CollisionPtr _parent);

      /// \brief Destructor
      public: virtual ~PluginHeightmapShape();

      // Documentation inerited.
      public: virtual void Init();
    };
  }
}
#endif
