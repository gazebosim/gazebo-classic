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
/* Desc: Heightmap collisionetry
 * Author: Nate Koenig, Andrew Howard
 * Date: 8 May 2003
 */

#include "common/Exception.hh"

#include "physics/bullet/bullet_inc.h"
#include "physics/bullet/BulletTypes.hh"
#include "physics/bullet/BulletCollision.hh"
#include "physics/bullet/BulletHeightmapShape.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletHeightmapShape::BulletHeightmapShape(CollisionPtr _parent)
    : HeightmapShape(_parent)
{
}

//////////////////////////////////////////////////
BulletHeightmapShape::~BulletHeightmapShape()
{
}

//////////////////////////////////////////////////
void BulletHeightmapShape::Init()
{
  HeightmapShape::Init();

  float maxHeight = this->GetMaxHeight();
  float minHeight = this->GetMinHeight();

  // This will force the Z-axis to be up
  int upIndex = 2;
  btVector3 localScaling(this->scale.x, this->scale.y, 1.0);

  this->heightFieldShape  = new btHeightfieldTerrainShape(
      this->vertSize,     // # of heights along width
      this->vertSize,     // # of height along height
      &this->heights[0],  // The heights
      1,                  // Height scaling
      minHeight,          // Min height
      maxHeight,          // Max height
      upIndex,            // Up axis
      PHY_FLOAT,
      false);             // Flip quad edges

  this->heightFieldShape->setUseDiamondSubdivision(true);
  this->heightFieldShape->setLocalScaling(localScaling);

  BulletCollisionPtr bParent;
  bParent = boost::dynamic_pointer_cast<BulletCollision>(this->collisionParent);

  bParent->SetCollisionShape(this->heightFieldShape, false);

  math::Pose pose;
  pose.pos.x = 0;
  pose.pos.y = 0;
  pose.pos.z = (maxHeight - minHeight) * 0.5;
  bParent->SetRelativePose(pose, false);
}
