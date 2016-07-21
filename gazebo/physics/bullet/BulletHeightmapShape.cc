/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include "gazebo/common/Exception.hh"
#include "gazebo/common/Assert.hh"

#include "gazebo/physics/HeightmapShapePrivate.hh"
#include "gazebo/physics/bullet/bullet_inc.h"
#include "gazebo/physics/bullet/BulletTypes.hh"
#include "gazebo/physics/bullet/BulletLink.hh"
#include "gazebo/physics/bullet/BulletMotionState.hh"
#include "gazebo/physics/bullet/BulletCollision.hh"
#include "gazebo/physics/bullet/BulletHeightmapShape.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletHeightmapShape::BulletHeightmapShape(CollisionPtr _parent)
    : HeightmapShape(_parent)
{
  // Bullet need the height values flipped in the y direction
  this->heightmapShapeDPtr->flipY = true;
}

//////////////////////////////////////////////////
BulletHeightmapShape::~BulletHeightmapShape()
{
}

//////////////////////////////////////////////////
void BulletHeightmapShape::Init()
{
  HeightmapShape::Init();

  float maxHeight = this->MaxHeight();
  float minHeight = this->MinHeight();

  // This will force the Z-axis to be up
  int upIndex = 2;
  btVector3 localScaling(this->heightmapShapeDPtr->scale.X(),
      this->heightmapShapeDPtr->scale.Y(), 1.0);

  this->heightFieldShape  = new btHeightfieldTerrainShape(
      // # of heights along width
      this->heightmapShapeDPtr->vertSize,
      // # of height along height
      this->heightmapShapeDPtr->vertSize,
      // The heights
      &this->heightmapShapeDPtr->heights[0],
      // Height scaling
      1,
      // Min height
      minHeight,
      // Max height
      maxHeight,
      // Up axis
      upIndex,
      PHY_FLOAT,
      // Flip quad edges
      false);

  this->heightFieldShape->setLocalScaling(localScaling);

  // This is suppose to match up with Ogre rendering a bit better.
  this->heightFieldShape->setUseZigzagSubdivision(true);

  // Get a pointer to the parent collision
  BulletCollisionPtr bParent;
  bParent = std::dynamic_pointer_cast<BulletCollision>(
      this->heightmapShapeDPtr->collisionParent);

  GZ_ASSERT(bParent != nullptr,
      "Bullet collision parent of a heightmap is null");

  bParent->SetCollisionShape(this->heightFieldShape, false);

  btVector3 min, max;
  this->heightFieldShape->getAabb(btTransform::getIdentity(), min, max);

  BulletLinkPtr bLink = std::dynamic_pointer_cast<BulletLink>(
      bParent->Parent());

  GZ_ASSERT(bLink != nullptr, "Bullet heightmap does not have a link.");

  BulletMotionStatePtr motionState = bLink->MotionState();

  GZ_ASSERT(motionState != nullptr, "Invalid motion state for heightmap.");

  btTransform tr;
  tr.setIdentity();
  tr.setOrigin(btVector3(0, 0, (maxHeight - minHeight) * 0.5));

  // Set the transform for the heightmap.
  motionState->setWorldTransform(tr);
}
