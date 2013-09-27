/*
 * Copyright 2013 Open Source Robotics Foundation
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
/* Desc: Heightmap collisionetry
 * Author: Nate Koenig, Andrew Howard
 * Date: 8 May 2003
 */

#include "gazebo/common/Exception.hh"

#include "gazebo/physics/simbody/simbody_inc.h"
#include "gazebo/physics/simbody/SimbodyTypes.hh"
#include "gazebo/physics/simbody/SimbodyCollision.hh"
#include "gazebo/physics/simbody/SimbodyHeightmapShape.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodyHeightmapShape::SimbodyHeightmapShape(CollisionPtr _parent)
    : HeightmapShape(_parent)
{
}

//////////////////////////////////////////////////
SimbodyHeightmapShape::~SimbodyHeightmapShape()
{
}

//////////////////////////////////////////////////
void SimbodyHeightmapShape::Init()
{
  HeightmapShape::Init();
}
