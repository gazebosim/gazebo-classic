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
/* Desc: Heightmap geometry
 * Author: Nate Keonig
 * Date: 12 May 2009
 */

#include <string.h>
#include <math.h>

#include "rendering/ogre.h"
#include "common/Image.hh"
#include "common/Exception.hh"

#include "rendering/Scene.hh"
#include "rendering/Heightmap.hh"

using namespace gazebo;
using namespace rendering;

//////////////////////////////////////////////////
Heightmap::Heightmap(Scene *scene_)
{
  this->scene = scene_;
}

//////////////////////////////////////////////////
Heightmap::~Heightmap()
{
  this->scene->GetManager()->destroyQuery(this->rayQuery);
}

//////////////////////////////////////////////////
float Heightmap::GetHeightAt(const math::Vector2d &pos)
{
  Ogre::Vector3 pos3(pos.x, this->terrainSize.z, pos.y);

  this->ray.setOrigin(pos3);
  this->rayQuery->setRay(this->ray);
  this->distToTerrain = 0;
  this->rayQuery->execute(this);

  return this->terrainSize.z - this->distToTerrain;
}

//////////////////////////////////////////////////
bool Heightmap::queryResult(Ogre::MovableObject * /*obj_*/,
                            Ogre::Real /*dist_*/)
{
  return false;
}

//////////////////////////////////////////////////
bool Heightmap::queryResult(Ogre::SceneQuery::WorldFragment * /*frag_*/,
                            Ogre::Real dist_)
{
  this->distToTerrain = dist_;
  return false;
}

//////////////////////////////////////////////////
void Heightmap::Load(std::string imageFilename,
                      std::string worldTexture,
                      std::string detailTexture,
                      math::Vector3 _terrainSize)
{
  std::ostringstream stream;
  unsigned int terrainVertSize;
  int tileSize;
  common::Image img;

  this->terrainSize = _terrainSize;

  // Use the image to get the size of the heightmap
  img.Load(imageFilename);

  // Width and height must be the same
  if (img.GetWidth() != img.GetHeight())
  {
    gzthrow("Heightmap image must be square\n");
  }

  terrainVertSize = img.GetWidth();

  float nf = static_cast<float>(log(terrainVertSize-1)/log(2));
  int ni = static_cast<int>(log(terrainVertSize-1)/log(2));

  // Make sure the heightmap image size is (2^n)+1 in size
  if (!math::equal(nf - ni, 0))
  {
    gzthrow("Heightmap image size must be (2^n)+1\n");
  }

  // Calculate a good tile size
  tileSize = static_cast<int>(pow(2, ni/2));

  if (tileSize <= 2)
  {
    tileSize = 4;
  }

  tileSize++;

  stream << "WorldTexture =" << worldTexture << "\n";
  // The detail texture
  stream << "DetailTexture =" << detailTexture << "\n";
  // number of times the detail texture will tile in a terrain tile
  stream << "DetailTile = 3\n";
  // Heightmap source
  stream << "PageSource = Heightmap\n";
  // Heightmap-source specific settings
  stream << "Heightmap.image =" << imageFilename << "\n";
  // How large is a page of tiles (in vertices)? Must be (2^n)+1
  stream << "PageSize =" << terrainVertSize << "\n";
  // How large is each tile? Must be (2^n)+1 and be smaller than PageSize
  stream << "TileSize =" << tileSize << "\n";
  // The maximum error allowed when determining which LOD to use
  stream << "MaxPixelError = 4\n";
  // The size of a terrain page, in world units
  stream << "PageWorldX =" << this->terrainSize.x << "\n";
  stream << "PageWorldZ =" << this->terrainSize.y << "\n";
  // Maximum height of the terrain
  stream << "MaxHeight ="<< this->terrainSize.z << "\n";
  // Upper LOD limit
  stream << "MaxMipMapLevel = 2\n";

  // Create a data stream for loading the terrain into Ogre
  char *mstr = strdup(stream.str().c_str());

  Ogre::DataStreamPtr dataStream(
    new Ogre::MemoryDataStream(mstr, strlen(mstr)));

  // Set the static terrain in Ogre
  this->scene->GetManager()->setWorldGeometry(dataStream);

  // HACK to make the terrain oriented properly
  Ogre::SceneNode *tnode = this->scene->GetManager()->getSceneNode("Terrain");
  tnode->pitch(Ogre::Degree(90));
  tnode->translate(
      Ogre::Vector3(-this->terrainSize.x*0.5, this->terrainSize.y*0.5, 0));

  // Setup the ray scene query, which is used to determine the heights of
  // the vertices for ODE
  this->ray = Ogre::Ray(Ogre::Vector3::ZERO, Ogre::Vector3::NEGATIVE_UNIT_Y);
  this->rayQuery = this->scene->GetManager()->createRayQuery(this->ray);
  this->rayQuery->setQueryTypeMask(
      Ogre::SceneManager::WORLD_GEOMETRY_TYPE_MASK);
  this->rayQuery->setWorldFragmentType(
      Ogre::SceneQuery::WFT_SINGLE_INTERSECTION);

  free(mstr);
}



