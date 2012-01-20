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

#include <string.h>
#include <math.h>

#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>

#include <iostream>

#include "common/Image.hh"
#include "common/Exception.hh"
#include "Link.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
BulletHeightmapCollision::BulletHeightmapCollision(Link *_body)
    : BulletCollision(body)
{
  common::Param::Begin(&this->parameters);
  this->imageFilenameP = new common::ParamT<std::string>("image", "", 1);
  this->worldTextureP = new common::ParamT<std::string>("worldTexture", "", 0);
  this->detailTextureP =
    new common::ParamT<std::string>("detailTexture", "", 0);
  this->sizeP = new common::ParamT<math::Vector3>("size",
      math::Vector3(10, 10, 10), 0);
  this->offsetP = new common::ParamT<math::Vector3>("offset",
      math::Vector3(0, 0, 0), 0);
  common::Param::End();

  this->ogreHeightmap = new OgreHeightmap();
}


//////////////////////////////////////////////////
BulletHeightmapCollision::~BulletHeightmapCollision()
{
  delete this->imageFilenameP;
  delete this->worldTextureP;
  delete this->detailTextureP;
  delete this->sizeP;
  delete this->offsetP;

  delete this->ogreHeightmap;
}

//////////////////////////////////////////////////
void BulletHeightmapCollision::Update()
{
  BulletCollision::Update();
}

//////////////////////////////////////////////////
void BulletHeightmapCollision::FillHeightMap()
{
  unsigned int x, y;
  float *heights = new float[this->width * this->height];
  float maxHeight = -FLT_MAX;

  math::Vector3 scale = this->terrainSize / this->width;

  // Iterate over all the verices
  for (y = 0; y < this->height; y++)
  {
    for (x = 0; x < this->width; x++)
    {
      // Find the height at a vertex
      heights[y*this->width + x] = this->ogreHeightmap->GetHeightAt(
          math::Vector2<float>(x*scale.x, y*scale.y));

      if (heights[y*this->width + x] > maxHeight)
        maxHeight = heights[y*this->width + x];
    }
  }

  delete this->collisionShape;

  // This will force the Z-axis to be up
  int upIndex = 2;

  btmath::Vector3 localScaling(this->terrainSize.x, this->terrainSize.y,
                         this->terrainSize.z);

  this->heightFieldShape  = new btHeightfieldTerrainShape(this->width,
      this->height, heights, maxHeight, upIndex, true, false);

  this->heightFieldShape->setUseDiamondSubdivision(true);
  this->heightFieldShape->setLocalScaling(localScaling);

  this->collisionShape = this->heightFieldShape;
}

//////////////////////////////////////////////////
void BulletHeightmapCollision::Load(common::XMLConfigNode *_node)
{
  Image tmpImage;

  this->imageFilenameP->Load(_node);
  this->worldTextureP->Load(_node);
  this->detailTextureP->Load(_node);
  this->sizeP->Load(_node);
  this->offsetP->Load(_node);

  // Use the image to get the size of the heightmap
  tmpImage.Load((**this->imageFilenameP));

  // Width and height must be the same
  if (tmpImage.GetWidth() != tmpImage.GetHeight())
  {
    gzthrow("Heightmap image must be square\n");
  }

  this->width = this->height = tmpImage.GetWidth();

  this->terrainSize = (**this->sizeP);

  // Step 1: Create the Ogre height map: Performs a ray scene query
  this->ogreHeightmap->Load((**this->imageFilenameP), (**this->worldTextureP),
      (**this->detailTextureP), this->terrainSize);

  // Step 2: Fill the bullet heightmap
  this->FillHeightMap();

  BulletCollision::Load(_node);
}

//////////////////////////////////////////////////
void BulletHeightmapCollision::Save(std::string &_prefix, std::ostream &_stream)
{
  BulletCollision::Save(_prefix, _stream);
  _stream << _prefix << *(this->imageFilenameP) << "\n";
  _stream << _prefix << *(this->worldTextureP) << "\n";
  _stream << _prefix << *(this->detailTextureP) << "\n";
  _stream << _prefix << *(this->sizeP) << "\n";
  _stream << _prefix << *(this->offsetP) << "\n";
}


