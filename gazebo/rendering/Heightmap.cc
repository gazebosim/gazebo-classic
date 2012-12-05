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
/* Desc: Heightmap geometry
 * Author: Nate Koenig
 * Date: 12 May 2009
 */

#include <string.h>
#include <math.h>

#include <Terrain/OgreTerrain.h>
#include <Terrain/OgreTerrainGroup.h>

#include "common/Common.hh"
#include "math/Helpers.hh"
#include "rendering/ogre_gazebo.h"
#include "common/Exception.hh"

#include "rendering/Scene.hh"
#include "rendering/Light.hh"
#include "rendering/Conversions.hh"
#include "rendering/Heightmap.hh"

using namespace gazebo;
using namespace rendering;

//////////////////////////////////////////////////
Heightmap::Heightmap(ScenePtr _scene)
{
  this->scene = _scene;
}

//////////////////////////////////////////////////
Heightmap::~Heightmap()
{
  this->scene.reset();
}

//////////////////////////////////////////////////
void Heightmap::LoadFromMsg(ConstVisualPtr &_msg)
{
  msgs::Set(this->heightImage, _msg->geometry().heightmap().image());
  this->terrainSize = msgs::Convert(_msg->geometry().heightmap().size());
  this->terrainOrigin = msgs::Convert(_msg->geometry().heightmap().origin());

  for (int i = 0; i < _msg->geometry().heightmap().texture_size(); ++i)
  {
    this->diffuseTextures.push_back(common::find_file(
        _msg->geometry().heightmap().texture(i).diffuse()));
    this->normalTextures.push_back(common::find_file(
        _msg->geometry().heightmap().texture(i).normal()));
    this->worldSizes.push_back(
        _msg->geometry().heightmap().texture(i).size());
  }

  for (int i = 0; i < _msg->geometry().heightmap().blend_size(); ++i)
  {
    this->blendHeight.push_back(
        _msg->geometry().heightmap().blend(i).min_height());
    this->blendFade.push_back(
        _msg->geometry().heightmap().blend(i).fade_dist());
  }

  this->Load();
}

//////////////////////////////////////////////////
void Heightmap::Load()
{
  this->terrainGlobals = new Ogre::TerrainGlobalOptions();

  if (this->heightImage.GetWidth() != this->heightImage.GetHeight() ||
      !math::isPowerOfTwo(this->heightImage.GetWidth() - 1))
  {
    gzthrow("Heightmap image size must be square, with a size of 2^n+1\n");
  }

  this->imageSize = this->heightImage.GetWidth();
  this->maxPixel = this->heightImage.GetMaxColor().r;

  if (math::equal(this->maxPixel, 0.0))
    this->maxPixel = 1.0;

  // Create terrain group, which holds all the individual terrain instances.
  // Param 1: Pointer to the scene manager
  // Param 2: Alignment plane
  // Param 3: Number of vertices along one edge of the terrain (2^n+1).
  //          Terrains must be square, with each side a power of 2 in size
  // Param 4: World size of each terrain instance, in meters.
  this->terrainGroup = new Ogre::TerrainGroup(
      this->scene->GetManager(), Ogre::Terrain::ALIGN_X_Y,
      this->imageSize, this->terrainSize.x);

  this->terrainGroup->setFilenameConvention(
      Ogre::String("gazebo_terrain"), Ogre::String("dat"));

  this->terrainGroup->setOrigin(Conversions::Convert(this->terrainOrigin));

  this->ConfigureTerrainDefaults();

  for (int x = 0; x <= 0; ++x)
    for (int y = 0; y <= 0; ++y)
      this->DefineTerrain(x, y);

  // sync load since we want everything in place when we start
  this->terrainGroup->loadAllTerrains(true);

  // Calculate blend maps
  if (this->terrainsImported)
  {
    Ogre::TerrainGroup::TerrainIterator ti =
      this->terrainGroup->getTerrainIterator();
    while (ti.hasMoreElements())
    {
      Ogre::Terrain *t = ti.getNext()->instance;
      this->InitBlendMaps(t);
    }
  }

  this->terrainGroup->freeTemporaryResources();
}

///////////////////////////////////////////////////
void Heightmap::ConfigureTerrainDefaults()
{
  // Configure global

  // MaxPixelError: Decides how precise our terrain is going to be.
  // A lower number will mean a more accurate terrain, at the cost of
  // performance (because of more vertices)
  this->terrainGlobals->setMaxPixelError(2);

  // CompositeMapDistance: decides how far the Ogre terrain will render
  // the lightmapped terrain.
  this->terrainGlobals->setCompositeMapDistance(1000);

  // Get the first directional light
  LightPtr directionalLight;
  for (unsigned int i = 0; i < this->scene->GetLightCount(); ++i)
  {
    LightPtr light = this->scene->GetLight(i);
    if (light->GetType() == "directional")
    {
      directionalLight = light;
      break;
    }
  }

  this->terrainGlobals->setCompositeMapAmbient(
      this->scene->GetManager()->getAmbientLight());

  // Important to set these so that the terrain knows what to use for
  // derived (non-realtime) data
  if (directionalLight)
  {
    this->terrainGlobals->setLightMapDirection(
        Conversions::Convert(directionalLight->GetDirection()));

    this->terrainGlobals->setCompositeMapDiffuse(
        Conversions::Convert(directionalLight->GetDiffuseColor()));
  }
  else
  {
    this->terrainGlobals->setLightMapDirection(Ogre::Vector3(0, 0, -1));
    this->terrainGlobals->setCompositeMapDiffuse(
        Ogre::ColourValue(.6, .6, .6, 1));
  }

  // Configure default import settings for if we use imported image
  Ogre::Terrain::ImportData &defaultimp =
    this->terrainGroup->getDefaultImportSettings();

  defaultimp.terrainSize = this->imageSize;
  defaultimp.worldSize = this->terrainSize.x;

  defaultimp.inputScale = this->terrainSize.z / this->maxPixel;

  defaultimp.minBatchSize = 33;
  defaultimp.maxBatchSize = 65;

  // textures. The default material generator takes two materials per layer.
  //    1. diffuse_specular - diffuse texture with a specular map in the
  //    alpha channel
  //    2. normal_height - normal map with a height map in the alpha channel
  {
    // number of texture layers
    defaultimp.layerList.resize(this->diffuseTextures.size());

    // The worldSize decides how big each splat of textures will be.
    // A smaller value will increase the resolution
    for (unsigned int i = 0; i < this->diffuseTextures.size(); ++i)
    {
      defaultimp.layerList[i].worldSize = this->worldSizes[i];
      defaultimp.layerList[i].textureNames.push_back(this->diffuseTextures[i]);
      defaultimp.layerList[i].textureNames.push_back(this->normalTextures[i]);
    }
  }
}

/////////////////////////////////////////////////
void Heightmap::DefineTerrain(int x, int y)
{
  Ogre::String filename = this->terrainGroup->generateFilename(x, y);

  if (Ogre::ResourceGroupManager::getSingleton().resourceExists(
        this->terrainGroup->getResourceGroup(), filename))
  {
    this->terrainGroup->defineTerrain(x, y);
  }
  else
  {
    Ogre::Image img;
    bool flipX = x % 2 != 0;
    bool flipY = y % 2 != 0;

    unsigned char *data = NULL;
    unsigned int count = 0;
    this->heightImage.GetData(&data, count);

    if (this->heightImage.GetPixelFormat() == common::Image::L_INT8)
    {
      img.loadDynamicImage(data, this->heightImage.GetWidth(),
          this->heightImage.GetHeight(), Ogre::PF_L8);
    }
    else if (this->heightImage.GetPixelFormat() == common::Image::RGBA_INT8)
    {
      img.loadDynamicImage(data, this->heightImage.GetWidth(),
          this->heightImage.GetHeight(), Ogre::PF_R8G8B8A8);
    }
    else if (this->heightImage.GetPixelFormat() == common::Image::RGB_INT8)
    {
      img.loadDynamicImage(data, this->heightImage.GetWidth(),
          this->heightImage.GetHeight(), Ogre::PF_R8G8B8);
    }
    else
    {
      gzerr << "Unable to handle image format["
            << this->heightImage.GetPixelFormat() << "]\n";
    }

    if (flipX)
      img.flipAroundY();
    if (flipY)
      img.flipAroundX();

    this->terrainGroup->defineTerrain(x, y, &img);
    this->terrainsImported = true;

    // delete [] data;
  }
}

/////////////////////////////////////////////////
bool Heightmap::InitBlendMaps(Ogre::Terrain *_terrain)
{
  if (!_terrain)
  {
    std::cerr << "Invalid  terrain\n";
    return false;
  }

  Ogre::Real val, height;
  unsigned int i = 0;

  std::vector<Ogre::TerrainLayerBlendMap *> blendMaps;
  std::vector<float*> pBlend;

  // Create the blend maps
  for (i = 0; i < this->blendHeight.size(); ++i)
  {
    blendMaps.push_back(_terrain->getLayerBlendMap(i+1));
    pBlend.push_back(blendMaps[i]->getBlendPointer());
  }

  // Set the blend values based on the height of the terrain
  for (Ogre::uint16 y = 0; y < _terrain->getLayerBlendMapSize(); ++y)
  {
    for (Ogre::uint16 x = 0; x < _terrain->getLayerBlendMapSize(); ++x)
    {
      Ogre::Real tx, ty;

      blendMaps[0]->convertImageToTerrainSpace(x, y, &tx, &ty);
      height = _terrain->getHeightAtTerrainPosition(tx, ty);

      for (i = 0; i < this->blendHeight.size(); ++i)
      {
        val = (height - this->blendHeight[i]) / this->blendFade[i];
        val = Ogre::Math::Clamp(val, (Ogre::Real)0, (Ogre::Real)1);
        *pBlend[i]++ = val;
      }
    }
  }

  // Make sure the blend maps are properly updated
  for (i = 0; i < blendMaps.size(); ++i)
  {
    blendMaps[i]->dirty();
    blendMaps[i]->update();
  }

  return true;
}

/////////////////////////////////////////////////
double Heightmap::GetHeight(double _x, double _y, double _z)
{
  Ogre::TerrainGroup::RayResult result = this->terrainGroup->rayIntersects(
      Ogre::Ray(Ogre::Vector3(_x, _y, _z), Ogre::Vector3(0, 0, -1)));

  if (result.hit)
    return result.position.z;
  else
    return 0;
}
