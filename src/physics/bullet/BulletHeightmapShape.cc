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
/* Desc: Heightmap collisionetry
 * Author: Nate Keonig, Andrew Howard
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
  unsigned int x, y;
  float maxHeight = -FLT_MAX;
  float minHeight = FLT_MAX;

  int imgHeight = this->img.GetHeight();
  int imgWidth = this->img.GetWidth();
  unsigned int pitch = this->img.GetPitch();
  unsigned int bpp = pitch / imgWidth;

  unsigned char *data = NULL;
  unsigned int count;
  this->heights.resize(imgWidth * imgHeight);

  this->img.GetData(&data, count);

  math::Vector3 terrainSize = this->sdf->GetValueVector3("size");

  math::Vector3 scale;
  scale.x = terrainSize.x / imgWidth;
  scale.y = terrainSize.y / imgHeight;
  if (math::equal(this->img.GetMaxColor().R(), 0))
    scale.z = terrainSize.z;
  else
    scale.z = terrainSize.z / this->img.GetMaxColor().R();

  // Iterate over all the verices
  for (y = 0; y < imgHeight; ++y)
  {
    for (x = 0; x < imgWidth; ++x)
    {
      float h = static_cast<int>(data[y * pitch + x * bpp]) / 255.0;
      h *= scale.z;

      // Find the height at a vertex
      this->heights[(imgHeight-y-1) * imgWidth + (x)] = h;
        
      if (h > maxHeight)
        maxHeight = h;
      if (h < minHeight)
        minHeight = h;
    }
  }


  // This will force the Z-axis to be up
  int upIndex = 2;
  btVector3 localScaling(scale.x, scale.y, 1.0);

  minHeight = 0;
  maxHeight = 10;

  this->heightFieldShape  = new btHeightfieldTerrainShape(
      imgWidth, // # of heights along width
      imgHeight,  // # of height along height
      &this->heights[0], // The heights
      1,  // Height scaling
      minHeight, // Min height
      maxHeight, // Max height
      upIndex, // Up axis 
      PHY_FLOAT,
      false); // Flip quad edges

  this->heightFieldShape->setUseDiamondSubdivision(true);
  this->heightFieldShape->setLocalScaling(localScaling);

  BulletCollisionPtr bParent;
  bParent = boost::shared_dynamic_cast<BulletCollision>(this->collisionParent);

  bParent->SetCollisionShape(this->heightFieldShape);

  math::Pose pose;
  pose.pos.x = 0;
  pose.pos.y = 0;
  pose.pos.z = (maxHeight - minHeight) * 0.5;
  bParent->SetRelativePose(pose, false);
}
