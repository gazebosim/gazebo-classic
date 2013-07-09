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
/* Desc: Trimesh shape
 * Author: Nate Koenig
 * Date: 21 May 2009
 */

#include "gazebo/common/Mesh.hh"

#include "gazebo/physics/simbody/SimbodyTypes.hh"
#include "gazebo/physics/simbody/SimbodyCollision.hh"
#include "gazebo/physics/simbody/SimbodyPhysics.hh"
#include "gazebo/physics/simbody/SimbodyTrimeshShape.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodyTrimeshShape::SimbodyTrimeshShape(CollisionPtr _parent)
  : TrimeshShape(_parent)
{
}


//////////////////////////////////////////////////
SimbodyTrimeshShape::~SimbodyTrimeshShape()
{
}

//////////////////////////////////////////////////
void SimbodyTrimeshShape::Load(sdf::ElementPtr _sdf)
{
  TrimeshShape::Load(_sdf);
}

//////////////////////////////////////////////////
void SimbodyTrimeshShape::Init()
{
  TrimeshShape::Init();

  SimbodyCollisionPtr bParent =
    boost::shared_static_cast<SimbodyCollision>(this->collisionParent);
}
