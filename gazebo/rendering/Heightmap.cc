/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include <memory>

#include <string.h>
#include <math.h>

#include <ignition/math/Color.hh>
#include <ignition/math/Matrix4.hh>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/Dem.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/HeightmapData.hh"
#include "gazebo/common/SystemPaths.hh"
#include "gazebo/transport/TransportIface.hh"
#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/RTShaderSystem.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Light.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/RenderEngine.hh"

#include "gazebo/rendering/Heightmap.hh"
#include "gazebo/rendering/HeightmapPrivate.hh"

using namespace gazebo;
using namespace rendering;

const double HeightmapPrivate::loadRadiusFactor = 1.0;
const double HeightmapPrivate::holdRadiusFactor = 1.15;
const boost::filesystem::path HeightmapPrivate::pagingDirname = "paging";
const boost::filesystem::path HeightmapPrivate::hashFilename = "gzterrain.SHA1";

static std::string glslVersion = "130";
static std::string vpInStr = "in";
static std::string vpOutStr = "out";
static std::string fpInStr = "in";
static std::string fpOutStr = "out";
static std::string textureStr = "texture";

//////////////////////////////////////////////////
Heightmap::Heightmap(ScenePtr _scene)
  : dataPtr(new HeightmapPrivate)
{
  this->dataPtr->scene = _scene;

  this->dataPtr->terrainIdx = 0;
  this->dataPtr->useTerrainPaging = false;
  this->dataPtr->terrainHashChanged = true;
  this->dataPtr->terrainsImported = true;

  this->dataPtr->gzPagingDir =
      common::SystemPaths::Instance()->GetLogPath() /
      this->dataPtr->pagingDirname;
}

//////////////////////////////////////////////////
Heightmap::~Heightmap()
{
  this->dataPtr->scene.reset();

  if (this->dataPtr->terrainPaging)
  {
    OGRE_DELETE this->dataPtr->terrainPaging;
    this->dataPtr->pageManager->destroyWorld(this->dataPtr->world);
    OGRE_DELETE this->dataPtr->pageManager;
  }
  else
  {
    this->dataPtr->terrainGroup->removeAllTerrains();

    OGRE_DELETE this->dataPtr->terrainGroup;
    this->dataPtr->terrainGroup = nullptr;
  }

  OGRE_DELETE this->dataPtr->terrainGlobals;
  this->dataPtr->terrainGlobals = nullptr;
}

//////////////////////////////////////////////////
void Heightmap::LoadFromMsg(ConstVisualPtr &_msg)
{
  this->dataPtr->terrainSize =
      msgs::ConvertIgn(_msg->geometry().heightmap().size());
  this->dataPtr->terrainOrigin =
      msgs::ConvertIgn(_msg->geometry().heightmap().origin());

  for (int i = 0; i < _msg->geometry().heightmap().texture_size(); ++i)
  {
    std::string diffusePath = common::find_file(
        _msg->geometry().heightmap().texture(i).diffuse());
    std::string normalPath = common::find_file(
        _msg->geometry().heightmap().texture(i).normal());

    RenderEngine::Instance()->AddResourcePath(diffusePath);
    RenderEngine::Instance()->AddResourcePath(normalPath);

    this->dataPtr->diffuseTextures.push_back(diffusePath);
    this->dataPtr->normalTextures.push_back(normalPath);
    this->dataPtr->worldSizes.push_back(
        _msg->geometry().heightmap().texture(i).size());
  }

  for (int i = 0; i < _msg->geometry().heightmap().blend_size(); ++i)
  {
    this->dataPtr->blendHeight.push_back(
        _msg->geometry().heightmap().blend(i).min_height());
    this->dataPtr->blendFade.push_back(
        _msg->geometry().heightmap().blend(i).fade_dist());
  }

  if (_msg->geometry().heightmap().has_use_terrain_paging())
  {
    this->dataPtr->useTerrainPaging =
        _msg->geometry().heightmap().use_terrain_paging();
  }

  if (_msg->geometry().heightmap().has_filename())
  {
    std::string uri = _msg->geometry().heightmap().filename();
    if (!uri.empty())
    {
      this->dataPtr->filename = common::find_file(
          _msg->geometry().heightmap().filename());
      if (this->dataPtr->filename.empty())
      {
        gzerr << "Unable to find file "
              << _msg->geometry().heightmap().filename()
              << std::endl;
      }
    }
  }

  if (_msg->geometry().heightmap().has_sampling())
  {
    unsigned int s = _msg->geometry().heightmap().sampling();
    if (!ignition::math::isPowerOfTwo(s))
    {
      gzerr << "Heightmap sampling value must be a power of 2. "
            << "The default value of 2 will be used instead." << std::endl;
      this->dataPtr->sampling = 2u;
    }
    else
    {
      this->dataPtr->sampling = s;
    }
  }

  this->SetCastShadows(_msg->cast_shadows());

  this->Load();
}

//////////////////////////////////////////////////
Ogre::TerrainGroup *Heightmap::OgreTerrain() const
{
  return this->dataPtr->terrainGroup;
}

//////////////////////////////////////////////////
common::Image Heightmap::Image() const
{
  common::Image result;

  double height = 0.0;
  unsigned char *imageData = nullptr;

  /// \todo Support multiple terrain objects
  Ogre::Terrain *terrain = this->dataPtr->terrainGroup->getTerrain(0, 0);

  GZ_ASSERT(terrain != nullptr, "Unable to get a valid terrain pointer");

  double minHeight = terrain->getMinHeight();
  double maxHeight = terrain->getMaxHeight() - minHeight;

  // Get the number of vertices along one side of the terrain
  uint16_t size = terrain->getSize();

  // Create the image data buffer
  imageData = new unsigned char[size * size];

  // Get height data from all vertices
  for (uint16_t y = 0; y < size; ++y)
  {
    for (uint16_t x = 0; x < size; ++x)
    {
      // Normalize height value
      // Weird Ogre issue: terrain->getHeightAtPoint could return a value
      // larger than terrain->getMaxHeight().
      height = (std::min(terrain->getHeightAtPoint(x, y),
          terrain->getMaxHeight()) - minHeight) / maxHeight;

      GZ_ASSERT((height <= 1.0 + 1e-6),
          "Normalized terrain height > 1.0");
      GZ_ASSERT((height >= 0.0 - 1e-6),
          "Normalized terrain height < 0.0");

      // Scale height to a value between 0 and 255
      imageData[(size - y - 1)*size+x] =
        static_cast<unsigned char>(height * 255.0);
    }
  }

  result.SetFromData(imageData, size, size, common::Image::L_INT8);

  delete [] imageData;
  return result;
}

//////////////////////////////////////////////////
void Heightmap::SplitHeights(const std::vector<float> &_heightmap,
    const int _n, std::vector<std::vector<float> > &_v)
{
  // We support splitting the terrain in 4 or 16 pieces
  GZ_ASSERT(_n == 4 || _n == 16,
      "Invalid number of terrain divisions (it should be 4 or 16)");

  int count = 0;
  int width = sqrt(_heightmap.size());
  int newWidth = 1 + (width - 1) / sqrt(_n);

  // Memory allocation
  _v.resize(_n);

  for (int tileR = 0; tileR < sqrt(_n); ++tileR)
  {
    int tileIndex = tileR * sqrt(_n);
    for (int row = 0; row < newWidth - 1; ++row)
    {
      for (int tileC = 0; tileC < sqrt(_n); ++tileC)
      {
        for (int col = 0; col < newWidth - 1; ++col)
        {
          _v[tileIndex].push_back(_heightmap[count]);
          ++count;
        }
        // Copy last value into the last column
        _v[tileIndex].push_back(_v[tileIndex].back());

        tileIndex = tileR * sqrt(_n) +
            (tileIndex + 1) % static_cast<int>(sqrt(_n));
      }
      ++count;
    }
    // Copy the last row
    for (int i = 0; i < sqrt(_n); ++i)
    {
      tileIndex = tileR * sqrt(_n) + i;
      std::vector<float> lastRow(_v[tileIndex].end() - newWidth,
          _v[tileIndex].end());
      _v[tileIndex].insert(_v[tileIndex].end(),
          lastRow.begin(), lastRow.end());
    }
  }
}

//////////////////////////////////////////////////
void Heightmap::UpdateTerrainHash(const std::string &_hash,
    const boost::filesystem::path &_terrainDir)
{
  std::ofstream terrainHashFile;
  boost::filesystem::path terrainHashFullPath;

  // Create the subdirectories if they do not exist
  boost::filesystem::create_directories(_terrainDir);

  terrainHashFullPath = _terrainDir / this->dataPtr->hashFilename;

  // Update the terrain hash
  terrainHashFile.open(terrainHashFullPath.string().c_str());

  // Throw an error if we couldn't open the file for writing.
  if (terrainHashFile.is_open())
  {
    terrainHashFile << _hash;
    terrainHashFile.close();
  }
  else
  {
    gzerr << "Unable to open file for creating a terrain hash: [" +
        terrainHashFullPath.string() + "]\n";
  }
}

//////////////////////////////////////////////////
bool Heightmap::PrepareTerrain(
    const boost::filesystem::path &_terrainDirPath)
{
  std::string heightmapHash;
  boost::filesystem::path terrainHashFullPath;
  bool updateHash = true;

  // Compute the original heightmap's image.
  heightmapHash = common::get_sha1<std::vector<float> >(this->dataPtr->heights);

  // Check if the terrain hash exists
  terrainHashFullPath = _terrainDirPath / this->dataPtr->hashFilename;
  if (boost::filesystem::exists(terrainHashFullPath))
  {
    try
    {
      // Read the terrain hash
      std::ifstream in(terrainHashFullPath.string().c_str());
      std::stringstream buffer;
      buffer << in.rdbuf();
      std::string terrainHash(buffer.str());
      updateHash = terrainHash != heightmapHash;
    }
    catch(std::ifstream::failure &e)
    {
      gzerr << "Terrain paging error: Unable to read terrain hash\n";
    }
  }

  // Update the terrain hash and split the terrain into small pieces
  if (updateHash)
  {
    this->UpdateTerrainHash(heightmapHash, _terrainDirPath);
  }

  return updateHash;
}

//////////////////////////////////////////////////
void Heightmap::Load()
{
  if (this->dataPtr->terrainGlobals != nullptr)
    return;

  const Ogre::RenderSystemCapabilities *capabilities;
  Ogre::RenderSystemCapabilities::ShaderProfiles profiles;
  Ogre::RenderSystemCapabilities::ShaderProfiles::const_iterator iter;

  capabilities =
      Ogre::Root::getSingleton().getRenderSystem()->getCapabilities();
  Ogre::DriverVersion glVersion;
  glVersion.build = 0;
  glVersion.major = 3;
  glVersion.minor = 0;
  glVersion.release = 0;
  if (capabilities->isDriverOlderThanVersion(glVersion))
  {
    glslVersion = "120";
    vpInStr = "attribute";
    vpOutStr = "varying";
    fpInStr = "varying";
    textureStr = "texture2D";
  }

  // The terraingGroup is composed by a number of terrains (1 by default)
  int nTerrains = 1;

  this->dataPtr->terrainGlobals = new Ogre::TerrainGlobalOptions();

#if (OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 8) || \
    OGRE_VERSION_MAJOR > 1
  // Vertex compression breaks anything, e.g. Gpu laser, that tries to build
  // a depth map.
  this->dataPtr->terrainGlobals->setUseVertexCompressionWhenAvailable(false);
#endif

  // There is an issue with OGRE terrain LOD if heights are not relative to 0.
  // So we move the heightmap so that its min elevation = 0 before feeding to
  // ogre. It is later translated back by the setOrigin call.
  double minElevation = 0.0;

  // try loading heightmap data locally
  if (!this->dataPtr->filename.empty())
  {
    this->dataPtr->heightmapData = common::HeightmapDataLoader::LoadTerrainFile(
        this->dataPtr->filename);

    if (this->dataPtr->heightmapData)
    {
      // TODO add a virtual HeightmapData::GetMinElevation function to avoid the
      // ifdef check. i.e. heightmapSizeZ = GetMaxElevation - GetMinElevation
      double heightmapSizeZ = this->dataPtr->heightmapData->GetMaxElevation();
#ifdef HAVE_GDAL
      auto demData =
          dynamic_cast<common::Dem *>(this->dataPtr->heightmapData);
      if (demData)
      {
        heightmapSizeZ = heightmapSizeZ - demData->GetMinElevation();
        if (this->dataPtr->terrainSize == ignition::math::Vector3d::Zero)
        {
          this->dataPtr->terrainSize = ignition::math::Vector3d(
              demData->GetWorldWidth(), demData->GetWorldHeight(),
              heightmapSizeZ);
        }
        minElevation = demData->GetMinElevation();
      }
#endif

      // these params need to be the same as physics/HeightmapShape.cc
      // in order to generate consistent height data
      bool flipY = false;
      // sampling size along image width and height
      unsigned int vertSize = (this->dataPtr->heightmapData->GetWidth() *
          this->dataPtr->sampling) - this->dataPtr->sampling + 1;
      ignition::math::Vector3d scale;
      scale.X(this->dataPtr->terrainSize.X() / vertSize);
      scale.Y(this->dataPtr->terrainSize.Y() / vertSize);

      if (ignition::math::equal(heightmapSizeZ, 0.0))
        scale.Z(1.0);
      else
        scale.Z(fabs(this->dataPtr->terrainSize.Z()) / heightmapSizeZ);

      // Construct the heightmap lookup table
      std::vector<float> lookup;
      this->dataPtr->heightmapData->FillHeightMap(this->dataPtr->sampling,
          vertSize, this->dataPtr->terrainSize, scale, flipY, lookup);

      for (unsigned int y = 0; y < vertSize; ++y)
      {
        for (unsigned int x = 0; x < vertSize; ++x)
        {
          int index = (vertSize - y - 1) * vertSize + x;
          this->dataPtr->heights.push_back(lookup[index] - minElevation);
        }
      }

      this->dataPtr->dataSize = vertSize;
    }
  }

  // if heightmap fails to load locally, get the data from the server side
  if (this->dataPtr->heights.empty())
  {
    gzmsg << "Heightmap could not be loaded locally "
          << "(is it in the GAZEBO_RESOURCE_PATH?)- requesting data from "
          << "the server" << std::endl;

    msgs::Geometry geomMsg;

    boost::shared_ptr<msgs::Response> response = transport::request(
       this->dataPtr->scene->Name(), "heightmap_data");

    if (response->response() != "error" &&
        response->type() == geomMsg.GetTypeName())
    {
      geomMsg.ParseFromString(response->serialized_data());

      // Copy the height data.
      this->dataPtr->terrainSize = msgs::ConvertIgn(geomMsg.heightmap().size());
      this->dataPtr->heights.resize(geomMsg.heightmap().heights().size());
      memcpy(&this->dataPtr->heights[0], geomMsg.heightmap().heights().data(),
          sizeof(this->dataPtr->heights[0]) *
          geomMsg.heightmap().heights().size());

      this->dataPtr->dataSize = geomMsg.heightmap().width();
    }
  }

  if (this->dataPtr->heights.empty())
  {
    gzerr << "Failed to load terrain. Heightmap data is empty" << std::endl;
    return;
  }

  if (!ignition::math::isPowerOfTwo(this->dataPtr->dataSize - 1))
  {
    gzerr << "Heightmap image size must be square, with a size of 2^n+1"
        << std::endl;
    return;
  }

  boost::filesystem::path imgPath;
  boost::filesystem::path terrainName;
  boost::filesystem::path terrainDirPath;
  boost::filesystem::path prefix;
  if (!this->dataPtr->filename.empty())
  {
    // Get the full path of the image heightmap
    imgPath = this->dataPtr->filename;
    terrainName = imgPath.filename().stem();
    terrainDirPath = this->dataPtr->gzPagingDir / terrainName;

    // Add the top level terrain paging directory to the OGRE
    // ResourceGroupManager
    boost::filesystem::path actualPagingDir =
        this->dataPtr->gzPagingDir.make_preferred();
    if (!Ogre::ResourceGroupManager::getSingleton().resourceLocationExists(
          actualPagingDir.string(), "General"))
    {
      Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
          actualPagingDir.string(), "FileSystem", "General", true);
      Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(
          "General");
    }
  }

  // If the paging is enabled we modify the number of subterrains
  if (this->dataPtr->useTerrainPaging)
  {
    this->dataPtr->splitTerrain = true;
    nTerrains = this->dataPtr->numTerrainSubdivisions;
    prefix = terrainDirPath / "gazebo_terrain_cache";
  }
  else
  {
    // Note: ran into problems with LOD height glitches if heightmap size is
    // larger than 4096 so split it into chunks
    // Note: dataSize should be 2^n + 1
    if (this->dataPtr->maxPixelError > 0 && this->dataPtr->dataSize > 4096u)
    {
      this->dataPtr->splitTerrain = true;
      if (this->dataPtr->dataSize == 4097u)
        this->dataPtr->numTerrainSubdivisions = 4u;
      else
        this->dataPtr->numTerrainSubdivisions = 16u;
     nTerrains = this->dataPtr->numTerrainSubdivisions;

      gzmsg << "Large heightmap used with LOD. It will be subdivided into " <<
          this->dataPtr->numTerrainSubdivisions << " terrains." << std::endl;
    }
    prefix = terrainDirPath / "gazebo_terrain";
  }

  double sqrtN = sqrt(nTerrains);

  // Create terrain group, which holds all the individual terrain instances.
  // Param 1: Pointer to the scene manager
  // Param 2: Alignment plane
  // Param 3: Number of vertices along one edge of the terrain (2^n+1).
  //          Terrains must be square, with each side a power of 2 in size
  // Param 4: World size of each terrain instance, in meters.

  this->dataPtr->terrainGroup = new Ogre::TerrainGroup(
      this->dataPtr->scene->OgreSceneManager(), Ogre::Terrain::ALIGN_X_Y,
      1 + ((this->dataPtr->dataSize - 1) / sqrtN),
      this->dataPtr->terrainSize.X() / (sqrtN));

  this->dataPtr->terrainGroup->setFilenameConvention(
    Ogre::String(prefix.string()), Ogre::String("dat"));

  Ogre::Vector3 orig = Conversions::Convert(this->dataPtr->terrainOrigin);
  ignition::math::Vector3d origin(orig.x -0.5 * this->dataPtr->terrainSize.X() +
      0.5 * this->dataPtr->terrainSize.X() / sqrtN,
      orig.y -0.5 * this->dataPtr->terrainSize.X() +
      0.5 * this->dataPtr->terrainSize.X() / sqrtN,
      orig.z + minElevation);

  this->dataPtr->terrainGroup->setOrigin(Conversions::Convert(origin));
  this->ConfigureTerrainDefaults();

  if (!this->dataPtr->heights.empty())
  {
    UserCameraPtr userCam = this->dataPtr->scene->GetUserCamera(0);

    // Move the camera above the terrain only if the user did not modify the
    // camera position in the world file
    if (userCam && !userCam->IsCameraSetInWorldFile())
    {
      double h = *std::max_element(
        &this->dataPtr->heights[0],
        &this->dataPtr->heights[0] + this->dataPtr->heights.size());

      ignition::math::Vector3d camPos(5, -5, h + 200);
      ignition::math::Vector3d lookAt(0, 0, h);
      auto mat = ignition::math::Matrix4d::LookAt(camPos, lookAt);

      userCam->SetWorldPose(mat.Pose());
    }
  }

  this->dataPtr->terrainHashChanged = this->PrepareTerrain(terrainDirPath);

  if (this->dataPtr->useTerrainPaging)
  {
    if (this->dataPtr->terrainHashChanged)
    {
      // Split the terrain. Every subterrain will be saved on disk and paged
      this->SplitHeights(this->dataPtr->heights, nTerrains,
          this->dataPtr->subTerrains);
    }

    this->dataPtr->pageManager = OGRE_NEW Ogre::PageManager();
    this->dataPtr->pageManager->setPageProvider(
        &this->dataPtr->dummyPageProvider);

    // Add cameras
    for (unsigned int i = 0; i < this->dataPtr->scene->CameraCount(); ++i)
    {
      this->dataPtr->pageManager->addCamera(
          this->dataPtr->scene->GetCamera(i)->OgreCamera());
    }
    for (unsigned int i = 0; i < this->dataPtr->scene->UserCameraCount();
        ++i)
    {
      this->dataPtr->pageManager->addCamera(
          this->dataPtr->scene->GetUserCamera(i)->OgreCamera());
    }

    this->dataPtr->terrainPaging =
        OGRE_NEW Ogre::TerrainPaging(this->dataPtr->pageManager);
    this->dataPtr->world = this->dataPtr->pageManager->createWorld();
    this->dataPtr->terrainPaging->createWorldSection(
        this->dataPtr->world, this->dataPtr->terrainGroup,
        this->dataPtr->loadRadiusFactor * this->dataPtr->terrainSize.X(),
        this->dataPtr->holdRadiusFactor * this->dataPtr->terrainSize.X(),
        0, 0, sqrtN - 1, sqrtN - 1);
  }

  gzmsg << "Loading heightmap: " << terrainName.string() << std::endl;
  common::Time time = common::Time::GetWallTime();

  for (int y = 0; y <= sqrtN - 1; ++y)
    for (int x = 0; x <= sqrtN - 1; ++x)
      this->DefineTerrain(x, y);

  // use gazebo shaders
  this->CreateMaterial();

  // Sync load since we want everything in place when we start
  this->dataPtr->terrainGroup->loadAllTerrains(true);

  gzmsg << "Heightmap loaded. Process took: "
        <<  (common::Time::GetWallTime() - time).Double()
        << " seconds" << std::endl;

  // Calculate blend maps
  if (this->dataPtr->terrainsImported)
  {
    Ogre::TerrainGroup::TerrainIterator ti =
      this->dataPtr->terrainGroup->getTerrainIterator();
    while (ti.hasMoreElements())
    {
      Ogre::Terrain *t = ti.getNext()->instance;
      this->InitBlendMaps(t);
    }
  }

  this->dataPtr->terrainGroup->freeTemporaryResources();

  // save the terrain once its loaded
  if (this->dataPtr->terrainsImported)
  {
    this->dataPtr->connections.push_back(
        event::Events::ConnectPreRender(
        std::bind(&Heightmap::SaveHeightmap, this)));
  }
}

///////////////////////////////////////////////////
void Heightmap::SaveHeightmap()
{
  if (this->dataPtr->terrainsImported &&
      !this->dataPtr->terrainGroup->isDerivedDataUpdateInProgress())
  {
    // check to see if all terrains have been loaded before saving
    Ogre::TerrainGroup::TerrainIterator ti =
      this->dataPtr->terrainGroup->getTerrainIterator();
    while (ti.hasMoreElements())
    {
      Ogre::Terrain *t = ti.getNext()->instance;
      if (!t->isLoaded())
        return;
    }

    // saving an ogre terrain data file can take quite some time for large dems.
    gzmsg << "Saving heightmap cache data to " << (this->dataPtr->gzPagingDir /
        boost::filesystem::path(this->dataPtr->filename).stem()).string()
        << std::endl;
    common::Time time = common::Time::GetWallTime();

    this->dataPtr->terrainGroup->saveAllTerrains(true);

    gzmsg << "Heightmap cache data saved. Process took: "
          << (common::Time::GetWallTime() - time).Double() << " seconds."
          << std::endl;

    this->dataPtr->terrainsImported = false;
    this->dataPtr->connections.clear();
  }
}

///////////////////////////////////////////////////
void Heightmap::ConfigureTerrainDefaults()
{
  // Configure global

  // MaxPixelError: Decides how precise our terrain is going to be.
  // A lower number will mean a more accurate terrain, at the cost of
  // performance (because of more vertices)
  this->dataPtr->terrainGlobals->setMaxPixelError(this->dataPtr->maxPixelError);

  // CompositeMapDistance: decides how far the Ogre terrain will render
  // the lightmapped terrain.
  this->dataPtr->terrainGlobals->setCompositeMapDistance(2000);

#if (OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 8) || \
    OGRE_VERSION_MAJOR > 1
  // Vertex compression breaks anything, e.g. Gpu laser, that tries to build
  // a depth map.
  this->dataPtr->terrainGlobals->setUseVertexCompressionWhenAvailable(false);
#endif

  // Get the first directional light
  LightPtr directionalLight;
  for (unsigned int i = 0; i < this->dataPtr->scene->LightCount(); ++i)
  {
    LightPtr light = this->dataPtr->scene->LightByIndex(i);
    if (light->Type() == "directional")
    {
      directionalLight = light;
      break;
    }
  }

  this->dataPtr->terrainGlobals->setSkirtSize(this->dataPtr->skirtLength);

  this->dataPtr->terrainGlobals->setCastsDynamicShadows(
        this->dataPtr->castShadows);

  this->dataPtr->terrainGlobals->setCompositeMapAmbient(
      this->dataPtr->scene->OgreSceneManager()->getAmbientLight());

  // Important to set these so that the terrain knows what to use for
  // derived (non-realtime) data
  if (directionalLight)
  {
    this->dataPtr->terrainGlobals->setLightMapDirection(
        Conversions::Convert(directionalLight->Direction()));

    auto const &ignDiffuse = directionalLight->DiffuseColor();
    this->dataPtr->terrainGlobals->setCompositeMapDiffuse(
        Conversions::Convert(ignDiffuse));
  }
  else
  {
    this->dataPtr->terrainGlobals->setLightMapDirection(
        Ogre::Vector3(0, 0, -1));
    this->dataPtr->terrainGlobals->setCompositeMapDiffuse(
        Ogre::ColourValue(.6, .6, .6, 1));
  }

  // Configure default import settings for if we use imported image
  Ogre::Terrain::ImportData &defaultimp =
    this->dataPtr->terrainGroup->getDefaultImportSettings();

  defaultimp.terrainSize = this->dataPtr->dataSize;
  defaultimp.worldSize = this->dataPtr->terrainSize.X();

  defaultimp.inputScale = 1.0;

  defaultimp.minBatchSize = 17;
  defaultimp.maxBatchSize = 65;

  // textures. The default material generator takes two materials per layer.
  //    1. diffuse_specular - diffuse texture with a specular map in the
  //    alpha channel
  //    2. normal_height - normal map with a height map in the alpha channel
  {
    // number of texture layers
    defaultimp.layerList.resize(this->dataPtr->diffuseTextures.size());

    // The worldSize decides how big each splat of textures will be.
    // A smaller value will increase the resolution
    for (unsigned int i = 0; i < this->dataPtr->diffuseTextures.size(); ++i)
    {
      defaultimp.layerList[i].worldSize = this->dataPtr->worldSizes[i];
      defaultimp.layerList[i].textureNames.push_back(
          this->dataPtr->diffuseTextures[i]);
      defaultimp.layerList[i].textureNames.push_back(
          this->dataPtr->normalTextures[i]);
    }
  }
}

/////////////////////////////////////////////////
void Heightmap::SetWireframe(const bool _show)
{
  Ogre::TerrainGroup::TerrainIterator ti =
    this->dataPtr->terrainGroup->getTerrainIterator();
  while (ti.hasMoreElements())
  {
    Ogre::Terrain *terrain = ti.getNext()->instance;
    GZ_ASSERT(terrain != nullptr, "Unable to get a valid terrain pointer");

    Ogre::Material *material = terrain->getMaterial().get();

    unsigned int techniqueCount, passCount;
    Ogre::Technique *technique;
    Ogre::Pass *pass;

    for (techniqueCount = 0; techniqueCount < material->getNumTechniques();
         ++techniqueCount)
    {
      technique = material->getTechnique(techniqueCount);

      for (passCount = 0; passCount < technique->getNumPasses(); ++passCount)
      {
        pass = technique->getPass(passCount);
        if (_show)
          pass->setPolygonMode(Ogre::PM_WIREFRAME);
        else
          pass->setPolygonMode(Ogre::PM_SOLID);
      }
    }
  }
}

/////////////////////////////////////////////////
void Heightmap::DefineTerrain(const int _x, const int _y)
{
  Ogre::String filename = this->dataPtr->terrainGroup->generateFilename(_x, _y);

  bool resourceExists =
      Ogre::ResourceGroupManager::getSingleton().resourceExists(
      this->dataPtr->terrainGroup->getResourceGroup(), filename);

  if (resourceExists && !this->dataPtr->terrainHashChanged)
  {
    gzmsg << "Loading heightmap cache data: " << filename << std::endl;

    this->dataPtr->terrainGroup->defineTerrain(_x, _y);
    this->dataPtr->terrainsImported = false;
  }
  else
  {
    if (this->dataPtr->splitTerrain)
    {
      // generate the subterrains if needed
      if (this->dataPtr->subTerrains.empty())
      {
        this->SplitHeights(this->dataPtr->heights,
            this->dataPtr->numTerrainSubdivisions,
            this->dataPtr->subTerrains);
      }

      this->dataPtr->terrainGroup->defineTerrain(_x, _y,
          &this->dataPtr->subTerrains[this->dataPtr->terrainIdx][0]);
      ++this->dataPtr->terrainIdx;
    }
    else
    {
      this->dataPtr->terrainGroup->defineTerrain(_x, _y,
          &this->dataPtr->heights[0]);
    }
  }
}

/////////////////////////////////////////////////
bool Heightmap::InitBlendMaps(Ogre::Terrain *_terrain)
{
  if (!_terrain)
  {
    gzerr << "Invalid terrain\n";
    return false;
  }

  // no blending to be done if there's only one texture or no textures at all.
  if (this->dataPtr->blendHeight.size() <= 1u ||
      this->dataPtr->diffuseTextures.size() <= 1u)
    return false;

  // Bounds check for following loop
  if (_terrain->getLayerCount() < this->dataPtr->blendHeight.size() + 1)
  {
      gzerr << "Invalid terrain, too few layers to initialize blend map\n";
      return false;
  }

  Ogre::Real val, height;
  unsigned int i = 0;

  std::vector<Ogre::TerrainLayerBlendMap *> blendMaps;
  std::vector<float*> pBlend;

  // Create the blend maps
  for (i = 0; i < this->dataPtr->blendHeight.size(); ++i)
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

      for (i = 0; i < this->dataPtr->blendHeight.size(); ++i)
      {
        val = (height - this->dataPtr->blendHeight[i]) /
            this->dataPtr->blendFade[i];
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
double Heightmap::Height(const double _x, const double _y, const double _z)
    const
{
  GZ_ASSERT(this->dataPtr->terrainGroup, "TerrainGroup pointer is NULL");

  Ogre::TerrainGroup::RayResult result =
      this->dataPtr->terrainGroup->rayIntersects(
      Ogre::Ray(Ogre::Vector3(_x, _y, _z), Ogre::Vector3(0, 0, -1)));

  if (result.hit)
    return result.position.z;
  else
  {
    return 0;
  }
}

/////////////////////////////////////////////////
Ogre::TerrainGroup::RayResult Heightmap::MouseHit(CameraPtr _camera,
    const ignition::math::Vector2i &_mousePos) const
{
  Ogre::Ray mouseRay = _camera->OgreCamera()->getCameraToViewportRay(
      static_cast<float>(_mousePos.X()) /
      _camera->OgreViewport()->getActualWidth(),
      static_cast<float>(_mousePos.Y()) /
      _camera->OgreViewport()->getActualHeight());

  // The terrain uses a special ray intersection test.
  return this->dataPtr->terrainGroup->rayIntersects(mouseRay);
}

/////////////////////////////////////////////////
bool Heightmap::Smooth(CameraPtr _camera,
                       const ignition::math::Vector2i &_mousePos,
                       const double _outsideRadius, const double _insideRadius,
                       const double _weight)
{
  Ogre::TerrainGroup::RayResult terrainResult =
    this->MouseHit(_camera, _mousePos);

  if (terrainResult.hit)
    this->ModifyTerrain(terrainResult.position, _outsideRadius, _insideRadius,
        _weight, "smooth");

  return terrainResult.hit;
}

/////////////////////////////////////////////////
bool Heightmap::Flatten(CameraPtr _camera,
                        const ignition::math::Vector2i &_mousePos,
                        const double _outsideRadius, const double _insideRadius,
                        const double _weight)
{
  Ogre::TerrainGroup::RayResult terrainResult =
    this->MouseHit(_camera, _mousePos);

  if (terrainResult.hit)
    this->ModifyTerrain(terrainResult.position, _outsideRadius,
        _insideRadius, _weight, "flatten");

  return terrainResult.hit;
}

/////////////////////////////////////////////////
bool Heightmap::Raise(CameraPtr _camera,
    const ignition::math::Vector2i &_mousePos,
    const double _outsideRadius, const double _insideRadius,
    const double _weight)
{
  // The terrain uses a special ray intersection test.
  Ogre::TerrainGroup::RayResult terrainResult =
    this->MouseHit(_camera, _mousePos);

  if (terrainResult.hit)
    this->ModifyTerrain(terrainResult.position, _outsideRadius,
       _insideRadius, _weight, "raise");

  return terrainResult.hit;
}

/////////////////////////////////////////////////
bool Heightmap::Lower(CameraPtr _camera,
    const ignition::math::Vector2i &_mousePos,
    const double _outsideRadius, const double _insideRadius,
    const double _weight)
{
  // The terrain uses a special ray intersection test.
  Ogre::TerrainGroup::RayResult terrainResult =
    this->MouseHit(_camera, _mousePos);

  if (terrainResult.hit)
    this->ModifyTerrain(terrainResult.position, _outsideRadius,
        _insideRadius, _weight, "lower");

  return terrainResult.hit;
}

/////////////////////////////////////////////////
double Heightmap::AvgHeight(const ignition::math::Vector3d &_pos,
    const double _radius) const
{
  GZ_ASSERT(this->dataPtr->terrainGroup, "TerrainGroup pointer is NULL");
  Ogre::Terrain *terrain = this->dataPtr->terrainGroup->getTerrain(0, 0);

  if (!terrain)
  {
    gzerr << "Invalid heightmap position [" << _pos << "]\n";
    return 0.0;
  }

  int size = static_cast<int>(terrain->getSize());

  Ogre::Vector3 pos;
  terrain->getTerrainPosition(Conversions::Convert(_pos), &pos);

  int startx = (pos.x - _radius) * size;
  int starty = (pos.y - _radius) * size;
  int endx = (pos.x + _radius) * size;
  int endy = (pos.y + _radius) * size;

  startx = std::max(startx, 0);
  starty = std::max(starty, 0);

  endx = std::min(endx, size);
  endy = std::min(endy, size);

  double sum = 0.0;
  int count = 0;
  for (int y = starty; y <= endy; ++y)
  {
    for (int x = startx; x <= endx; ++x)
    {
      sum += terrain->getHeightAtPoint(x, y);
      count++;
    }
  }

  return sum / count;
}

/////////////////////////////////////////////////
void Heightmap::ModifyTerrain(Ogre::Vector3 _pos, const double _outsideRadius,
    const double _insideRadius, const double _weight, const std::string &_op)
{
  GZ_ASSERT(this->dataPtr->terrainGroup, "TerrainGroup pointer is NULL");
  Ogre::Terrain *terrain = this->dataPtr->terrainGroup->getTerrain(0, 0);

  if (!terrain)
  {
    gzerr << "Invalid heightmap position [" << _pos << "]\n";
    return;
  }

  int size = static_cast<int>(terrain->getSize());

  Ogre::Vector3 pos;
  terrain->getTerrainPosition(_pos, &pos);

  int startx = (pos.x - _outsideRadius) * size;
  int starty = (pos.y - _outsideRadius) * size;
  int endx = (pos.x + _outsideRadius) * size;
  int endy = (pos.y + _outsideRadius) * size;

  startx = std::max(startx, 0);
  starty = std::max(starty, 0);

  endx = std::min(endx, size);
  endy = std::min(endy, size);

  double avgHeight = 0;

  if (_op == "flatten" || _op == "smooth")
    avgHeight = this->AvgHeight(Conversions::ConvertIgn(pos), _outsideRadius);

  for (int y = starty; y <= endy; ++y)
  {
    for (int x = startx; x <= endx; ++x)
    {
      double tsXdist = (x / static_cast<double>(size)) - pos.x;
      double tsYdist = (y / static_cast<double>(size))  - pos.y;

      double weight = 1.0;
      double dist = sqrt(tsYdist * tsYdist + tsXdist * tsXdist);

      if (dist > _insideRadius)
      {
        weight = ignition::math::clamp(dist / _outsideRadius, 0.0, 1.0);
        weight = 1.0 - (weight * weight);
      }

      float addedHeight = weight * _weight;
      float newHeight = terrain->getHeightAtPoint(x, y);

      if (_op == "raise")
        newHeight += addedHeight;
      else if (_op == "lower")
        newHeight -= addedHeight;
      else if (_op == "flatten")
      {
        if (newHeight < avgHeight)
          newHeight += addedHeight;
        else
          newHeight -= addedHeight;
      }
      else if (_op == "smooth")
      {
        if (newHeight < avgHeight)
          newHeight += addedHeight;
        else
          newHeight -= addedHeight;
      }
      else
        gzerr << "Unknown terrain operation[" << _op << "]\n";

      terrain->setHeightAtPoint(x, y, newHeight);
    }
  }
  terrain->dirty();
  terrain->update();
}

/////////////////////////////////////////////////
void Heightmap::SetupShadows(bool _enableShadows)
{
  // Assume we get a shader model 2 material profile
  Ogre::TerrainMaterialGeneratorA::SM2Profile *matProfile;

  Ogre::TerrainMaterialGeneratorPtr matGen =
      this->dataPtr->terrainGlobals->getDefaultMaterialGenerator();

  matProfile = static_cast<GzTerrainMatGen::SM2Profile*>(
      matGen->getActiveProfile());
  if (!matProfile)
  {
    // using custom material script so ignore setting shadows
    return;
  }

  matProfile->setLayerParallaxMappingEnabled(false);

  if (_enableShadows)
  {
    // Make sure PSSM is already setup
    matProfile->setReceiveDynamicShadowsEnabled(true);
    matProfile->setReceiveDynamicShadowsPSSM(
        RTShaderSystem::Instance()->GetPSSMShadowCameraSetup());
    matProfile->setReceiveDynamicShadowsDepth(true);
    matProfile->setReceiveDynamicShadowsLowLod(false);
  }
  else
  {
    matProfile->setReceiveDynamicShadowsPSSM(nullptr);
  }
}

/////////////////////////////////////////////////
void Heightmap::SetLOD(const unsigned int _value)
{
  this->dataPtr->maxPixelError = _value;
  if (this->dataPtr->terrainGlobals)
  {
    this->dataPtr->terrainGlobals->setMaxPixelError(
        this->dataPtr->maxPixelError);
  }
}

/////////////////////////////////////////////////
unsigned int Heightmap::LOD() const
{
  return static_cast<unsigned int>(this->dataPtr->maxPixelError);
}

/////////////////////////////////////////////////
void Heightmap::SetSkirtLength(const double _value)
{
  this->dataPtr->skirtLength = _value;
  if (this->dataPtr->terrainGlobals)
  {
    this->dataPtr->terrainGlobals->setSkirtSize(
        this->dataPtr->skirtLength);
  }
}

/////////////////////////////////////////////////
bool Heightmap::CastShadows() const
{
  return this->dataPtr->castShadows;
}

/////////////////////////////////////////////////
void Heightmap::SetCastShadows(const bool _value)
{
  this->dataPtr->castShadows = _value;
  if (this->dataPtr->terrainGlobals)
  {
    this->dataPtr->terrainGlobals->setCastsDynamicShadows(
        this->dataPtr->castShadows);
  }
}

/////////////////////////////////////////////////
double Heightmap::SkirtLength() const
{
  return this->dataPtr->skirtLength;
}

/////////////////////////////////////////////////
void Heightmap::SetMaterial(const std::string &_materialName)
{
  this->dataPtr->materialName = _materialName;
  if (!this->dataPtr->materialName.empty() && this->dataPtr->terrainGlobals)
    this->CreateMaterial();
}

/////////////////////////////////////////////////
std::string Heightmap::MaterialName() const
{
  return this->dataPtr->materialName;
}

/////////////////////////////////////////////////
void Heightmap::CreateMaterial()
{
  if (!this->dataPtr->materialName.empty())
  {
    // init custom material generator
    Ogre::TerrainMaterialGeneratorPtr terrainMaterialGenerator;
    TerrainMaterial *terrainMaterial = OGRE_NEW TerrainMaterial(
        this->dataPtr->materialName);
    if (this->dataPtr->splitTerrain)
      terrainMaterial->setGridSize(this->dataPtr->numTerrainSubdivisions);
    terrainMaterialGenerator.bind(terrainMaterial);
    this->dataPtr->terrainGlobals->setDefaultMaterialGenerator(
        terrainMaterialGenerator);
  }
  else
  {
    // use default material
    // RTSS PSSM shadows compatible terrain material
    if (!this->dataPtr->gzMatGen)
      this->dataPtr->gzMatGen = new GzTerrainMatGen();

    Ogre::TerrainMaterialGeneratorPtr ptr = Ogre::TerrainMaterialGeneratorPtr();
    ptr.bind(this->dataPtr->gzMatGen);

    this->dataPtr->terrainGlobals->setDefaultMaterialGenerator(ptr);

    this->SetupShadows(true);
  }
}

/////////////////////////////////////////////////
unsigned int Heightmap::TerrainSubdivisionCount() const
{
  return this->dataPtr->numTerrainSubdivisions;
}

/////////////////////////////////////////////////
/////////////////////////////////////////////////
// GzTerrainMatGen
/////////////////////////////////////////////////
/////////////////////////////////////////////////


/////////////////////////////////////////////////
GzTerrainMatGen::GzTerrainMatGen()
: TerrainMaterialGeneratorA()
{
  /// \TODO - This will have to be changed if TerrainMaterialGeneratorA
  /// ever supports more profiles than only CG

  // Add custom SM2Profile SPAM
  this->mProfiles.clear();

  this->mProfiles.push_back(OGRE_NEW SM2Profile(this, "SM2",
        "Profile for rendering on Shader Model 2 capable cards "
        "(RTSS depth shadows compatible)"));

  /// \TODO - check hardware capabilities & use fallbacks if required
  /// (more profiles needed)
  this->setActiveProfile(this->mProfiles[0]);
}

/////////////////////////////////////////////////
GzTerrainMatGen::~GzTerrainMatGen()
{
}

/////////////////////////////////////////////////
GzTerrainMatGen::SM2Profile::SM2Profile(
    Ogre::TerrainMaterialGenerator *_parent, const Ogre::String &_name,
    const Ogre::String &_desc)
: TerrainMaterialGeneratorA::SM2Profile(_parent, _name, _desc)
{
  this->mShaderGen = nullptr;
}

/////////////////////////////////////////////////
GzTerrainMatGen::SM2Profile::~SM2Profile()
{
  // Because the base SM2Profile has no virtual destructor:
  delete this->mShaderGen;
  this->mShaderGen = nullptr;
}

/////////////////////////////////////////////////
void GzTerrainMatGen::SM2Profile::addTechnique(
    const Ogre::MaterialPtr &_mat, const Ogre::Terrain *_terrain,
    TechniqueType _tt)
{
  // Initiate specialized mShaderGen
  // Ogre::GpuProgramManager &gmgr = Ogre::GpuProgramManager::getSingleton();

  Ogre::HighLevelGpuProgramManager &hmgr =
    Ogre::HighLevelGpuProgramManager::getSingleton();

  if (!this->mShaderGen)
  {
    // By default we use the GLSL shaders.
    if (hmgr.isLanguageSupported("glsl"))
    {
      this->mShaderGen = OGRE_NEW
        GzTerrainMatGen::SM2Profile::ShaderHelperGLSL();
    }
    else
    {
      gzthrow("No supported shader languages");
    }

    // Uncomment this to use cg shaders. I'm keeping the CG
    // shader for reference. There is some more code to switch, located
    // below, to enable CG shaders.
    // if (hmgr.isLanguageSupported("cg"))
    // {
    //   this->mShaderGen = OGRE_NEW
    //     // This will use Ogre's CG shader
    //     // Ogre::TerrainMaterialGeneratorA::SM2Profile::ShaderHelperCg();
    //     //
    //     // This will use our CG shader, which has terrain shadows
    //     GzTerrainMatGen::SM2Profile::ShaderHelperCg();
    // }

    // check SM3 features
    this->mSM3Available =
      Ogre::GpuProgramManager::getSingleton().isSyntaxSupported("ps_3_0");

#if OGRE_VERSION_MAJOR >= 1 && OGRE_VERSION_MINOR >= 8
    this->mSM4Available =
      Ogre::GpuProgramManager::getSingleton().isSyntaxSupported("ps_4_0");
#endif
  }

  // Unfortunately this doesn't work
  // Default implementation
  // TerrainMaterialGeneratorA::SM2Profile::addTechnique(mat, terrain, tt);

  // So we have to replicate the entire method:
  Ogre::Technique *tech = _mat->createTechnique();

  // Only supporting one pass
  Ogre::Pass *pass = tech->createPass();

  // Doesn't delegate to the proper method otherwise
  Ogre::HighLevelGpuProgramPtr vprog =
    ((GzTerrainMatGen::SM2Profile::ShaderHelperGLSL*)this->mShaderGen)
  // Use this line if running Ogre's CG shaders
  // ((TerrainMaterialGeneratorA::SM2Profile::ShaderHelperCg*)this->mShaderGen)
  // Use this line if running our CG shaders
  // ((GzTerrainMatGen::SM2Profile::ShaderHelperCg*)this->mShaderGen)
    ->generateVertexProgram(this, _terrain, _tt);

  // DEBUG: std::cout << "VertShader[" << vprog->getName() << "]:\n"
  //          << vprog->getSource() << "\n\n";

  Ogre::HighLevelGpuProgramPtr fprog =
    ((GzTerrainMatGen::SM2Profile::ShaderHelperGLSL*)this->mShaderGen)
  // Use this line if running Ogre's CG shaders
  // ((TerrainMaterialGeneratorA::SM2Profile::ShaderHelperCg*)this->mShaderGen)
  // Use this line if running our CG shaders
  // ((GzTerrainMatGen::SM2Profile::ShaderHelperCg*)this->mShaderGen)
    ->generateFragmentProgram(this, _terrain, _tt);

  // DEBUG: std::cout << "FragShader[" << fprog->getName() << "]:\n"
  //          << fprog->getSource() << "\n\n";

  pass->setVertexProgram(vprog->getName());
  pass->setFragmentProgram(fprog->getName());

  if (_tt == HIGH_LOD || _tt == RENDER_COMPOSITE_MAP)
  {
    // global normal map
    Ogre::TextureUnitState* tu = pass->createTextureUnitState();
    tu->setTextureName(_terrain->getTerrainNormalMap()->getName());
    tu->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);

    // global colour map
    if (_terrain->getGlobalColourMapEnabled() &&
        this->isGlobalColourMapEnabled())
    {
      tu = pass->createTextureUnitState(
          _terrain->getGlobalColourMap()->getName());
      tu->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
    }

    // light map
    if (this->isLightmapEnabled())
    {
      tu = pass->createTextureUnitState(_terrain->getLightmap()->getName());
      tu->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
    }

    // blend maps
    unsigned int maxLayers = this->getMaxLayers(_terrain);

    unsigned int numBlendTextures = std::min(
        _terrain->getBlendTextureCount(maxLayers),
        _terrain->getBlendTextureCount());

    unsigned int numLayers = std::min(
        maxLayers, static_cast<unsigned int>(_terrain->getLayerCount()));

    for (unsigned int i = 0; i < numBlendTextures; ++i)
    {
      tu = pass->createTextureUnitState(_terrain->getBlendTextureName(i));
      tu->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
    }

    // layer textures
    for (unsigned int i = 0; i < numLayers; ++i)
    {
      // diffuse / specular
      pass->createTextureUnitState(_terrain->getLayerTextureName(i, 0));

      // normal / height
      pass->createTextureUnitState(_terrain->getLayerTextureName(i, 1));
    }
  }
  else
  {
    // LOW_LOD textures
    // composite map
    Ogre::TextureUnitState *tu = pass->createTextureUnitState();
    tu->setTextureName(_terrain->getCompositeMap()->getName());
    tu->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
  }

  // Add shadow textures (always at the end)
  if (this->isShadowingEnabled(_tt, _terrain))
  {
    unsigned int numTextures = 1;

    if (this->getReceiveDynamicShadowsPSSM())
    {
      numTextures = this->getReceiveDynamicShadowsPSSM()->getSplitCount();
    }
    for (unsigned int i = 0; i < numTextures; ++i)
    {
      Ogre::TextureUnitState *tu = pass->createTextureUnitState();
      tu->setContentType(Ogre::TextureUnitState::CONTENT_SHADOW);
      tu->setTextureAddressingMode(Ogre::TextureUnitState::TAM_BORDER);
      tu->setTextureBorderColour(Ogre::ColourValue::White);
    }
  }
}

/////////////////////////////////////////////////
// generate() and generateForCompositeMap() are identical to
// TerrainMaterialGeneratorA implementation, the only reason for repeating
// them is that, unfortunately, addTechnique() is not declared virtual.
Ogre::MaterialPtr GzTerrainMatGen::SM2Profile::generate(
    const Ogre::Terrain *_terrain)
{
  // re-use old material if exists
  Ogre::MaterialPtr mat = _terrain->_getMaterial();

  if (mat.isNull())
  {
    Ogre::MaterialManager &matMgr = Ogre::MaterialManager::getSingleton();

    // it's important that the names are deterministic for a given terrain, so
    // use the terrain pointer as an ID
    const Ogre::String &matName = _terrain->getMaterialName();
    mat = matMgr.getByName(matName);

    if (mat.isNull())
    {
      mat = matMgr.create(matName,
          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    }
  }

  // clear everything
  mat->removeAllTechniques();

  // Automatically disable normal & parallax mapping if card cannot handle it
  // We do this rather than having a specific technique for it since it's
  // simpler.
  Ogre::GpuProgramManager &gmgr = Ogre::GpuProgramManager::getSingleton();

  if (!gmgr.isSyntaxSupported("ps_4_0") &&
      !gmgr.isSyntaxSupported("ps_3_0") &&
      !gmgr.isSyntaxSupported("ps_2_x") &&
      !gmgr.isSyntaxSupported("fp40") &&
      !gmgr.isSyntaxSupported("arbfp1"))
  {
    this->setLayerNormalMappingEnabled(false);
    this->setLayerParallaxMappingEnabled(false);
  }

  this->addTechnique(mat, _terrain, HIGH_LOD);

  // LOD
  if (this->mCompositeMapEnabled)
  {
    this->addTechnique(mat, _terrain, LOW_LOD);
    Ogre::Material::LodValueList lodValues;
    lodValues.push_back(
        Ogre::TerrainGlobalOptions::getSingleton().getCompositeMapDistance());

    mat->setLodLevels(lodValues);
    Ogre::Technique *lowLodTechnique = mat->getTechnique(1);
    lowLodTechnique->setLodIndex(1);
  }

  this->UpdateParams(mat, _terrain);

  return mat;
}

/////////////////////////////////////////////////
Ogre::MaterialPtr GzTerrainMatGen::SM2Profile::generateForCompositeMap(
    const Ogre::Terrain *_terrain)
{
  // re-use old material if exists
  Ogre::MaterialPtr mat = _terrain->_getCompositeMapMaterial();

  if (mat.isNull())
  {
    Ogre::MaterialManager &matMgr = Ogre::MaterialManager::getSingleton();

    // it's important that the names are deterministic for a given terrain, so
    // use the terrain pointer as an ID
    const Ogre::String &matName = _terrain->getMaterialName() + "/comp";

    mat = matMgr.getByName(matName);

    if (mat.isNull())
    {
      mat = matMgr.create(matName,
          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    }
  }

  // clear everything
  mat->removeAllTechniques();

  this->addTechnique(mat, _terrain, RENDER_COMPOSITE_MAP);

  this->UpdateParamsForCompositeMap(mat, _terrain);

  return mat;
}

/////////////////////////////////////////////////
void GzTerrainMatGen::SM2Profile::UpdateParams(const Ogre::MaterialPtr &_mat,
                  const Ogre::Terrain *_terrain)
{
  static_cast<GzTerrainMatGen::SM2Profile::ShaderHelperGLSL*>(
      this->mShaderGen)->updateParams(this, _mat, _terrain, false);
}

/////////////////////////////////////////////////
void GzTerrainMatGen::SM2Profile::UpdateParamsForCompositeMap(
    const Ogre::MaterialPtr &_mat, const Ogre::Terrain *_terrain)
{
  static_cast<GzTerrainMatGen::SM2Profile::ShaderHelperGLSL*>(
      this->mShaderGen)->updateParams(this, _mat, _terrain, true);
}

/////////////////////////////////////////////////
/////////////////////////////////////////////////
// GLSL Shader helper
/////////////////////////////////////////////////
/////////////////////////////////////////////////

/////////////////////////////////////////////////
Ogre::HighLevelGpuProgramPtr
GzTerrainMatGen::SM2Profile::ShaderHelperGLSL::generateVertexProgram(
    const SM2Profile *_prof, const Ogre::Terrain *_terrain,
    TechniqueType _tt)
{
  Ogre::HighLevelGpuProgramPtr ret =
    this->createVertexProgram(_prof, _terrain, _tt);

  Ogre::StringStream sourceStr;
  this->generateVertexProgramSource(_prof, _terrain, _tt, sourceStr);

  ret->setSource(sourceStr.str());
  ret->load();
  this->defaultVpParams(_prof, _terrain, _tt, ret);

  return ret;
}

/////////////////////////////////////////////////
Ogre::HighLevelGpuProgramPtr
GzTerrainMatGen::SM2Profile::ShaderHelperGLSL::generateFragmentProgram(
    const SM2Profile *_prof, const Ogre::Terrain *_terrain, TechniqueType _tt)
{
  Ogre::HighLevelGpuProgramPtr ret = this->createFragmentProgram(_prof,
      _terrain, _tt);

  Ogre::StringStream sourceStr;

  this->generateFragmentProgramSource(_prof, _terrain, _tt, sourceStr);

  ret->setSource(sourceStr.str());

  ret->load();

  this->defaultFpParams(_prof, _terrain, _tt, ret);

  Ogre::GpuProgramParametersSharedPtr params = ret->getDefaultParameters();
  params->setIgnoreMissingParams(false);

  Ogre::uint maxLayers = _prof->getMaxLayers(_terrain);
  Ogre::uint numBlendTextures = std::min(
      _terrain->getBlendTextureCount(maxLayers),
      _terrain->getBlendTextureCount());

  Ogre::uint numLayers = std::min(maxLayers,
      static_cast<Ogre::uint>(_terrain->getLayerCount()));

  int samplerCounter = 0;

  if (_tt == LOW_LOD)
    params->setNamedConstant("compositeMap", samplerCounter++);
  else
  {
    params->setNamedConstant("globalNormal", samplerCounter++);

    if (_terrain->getGlobalColourMapEnabled() &&
        _prof->isGlobalColourMapEnabled())
    {
      params->setNamedConstant("globalColourMap", samplerCounter++);
    }

    if (_prof->isLightmapEnabled())
      params->setNamedConstant("lightMap", samplerCounter++);

    for (Ogre::uint i = 0; i < numBlendTextures; ++i)
    {
      params->setNamedConstant("blendTex" +
          boost::lexical_cast<std::string>(i), samplerCounter++);
    }

    for (Ogre::uint i = 0; i < numLayers; ++i)
    {
      params->setNamedConstant("difftex" +
          boost::lexical_cast<std::string>(i), samplerCounter++);
      params->setNamedConstant("normtex" +
          boost::lexical_cast<std::string>(i), samplerCounter++);
    }
  }

  if (_prof->isShadowingEnabled(_tt, _terrain))
  {
    Ogre::uint numTextures = 1;
    if (_prof->getReceiveDynamicShadowsPSSM())
    {
      numTextures = _prof->getReceiveDynamicShadowsPSSM()->getSplitCount();
    }

    for (Ogre::uint i = 0; i < numTextures; ++i)
    {
      params->setNamedConstant("shadowMap" +
          boost::lexical_cast<std::string>(i), samplerCounter++);
    }
  }

  return ret;
}

/////////////////////////////////////////////////
void GzTerrainMatGen::SM2Profile::ShaderHelperGLSL::updateParams(
    const SM2Profile *_prof, const Ogre::MaterialPtr &_mat,
    const Ogre::Terrain *_terrain, bool _compositeMap)
{
  Ogre::Pass *p = _mat->getTechnique(0)->getPass(0);

  if (_compositeMap)
  {
    this->updateVpParams(_prof, _terrain, RENDER_COMPOSITE_MAP,
        p->getVertexProgramParameters());
    this->updateFpParams(_prof, _terrain, RENDER_COMPOSITE_MAP,
        p->getFragmentProgramParameters());
  }
  else
  {
    // high lod
    this->updateVpParams(_prof, _terrain, HIGH_LOD,
        p->getVertexProgramParameters());
    this->updateFpParams(_prof, _terrain, HIGH_LOD,
        p->getFragmentProgramParameters());

    if (_prof->isCompositeMapEnabled())
    {
      // low lod
      p = _mat->getTechnique(1)->getPass(0);
      this->updateVpParams(_prof, _terrain, LOW_LOD,
          p->getVertexProgramParameters());
      this->updateFpParams(_prof, _terrain, LOW_LOD,
          p->getFragmentProgramParameters());
    }
  }
}

/////////////////////////////////////////////////
void GzTerrainMatGen::SM2Profile::ShaderHelperGLSL::
generateVertexProgramSource(const SM2Profile *_prof,
    const Ogre::Terrain* _terrain, TechniqueType _tt,
    Ogre::StringStream &_outStream)
{
  this->generateVpHeader(_prof, _terrain, _tt, _outStream);

  if (_tt != LOW_LOD)
  {
    unsigned int maxLayers = _prof->getMaxLayers(_terrain);
    unsigned int numLayers = std::min(maxLayers,
        static_cast<unsigned int>(_terrain->getLayerCount()));

    for (unsigned int i = 0; i < numLayers; ++i)
      this->generateVpLayer(_prof, _terrain, _tt, i, _outStream);
  }

  this->generateVpFooter(_prof, _terrain, _tt, _outStream);
}

/////////////////////////////////////////////////
// This method is identical to
// TerrainMaterialGeneratorA::SM2Profile::ShaderHelperGLSL::generateVpHeader()
// but is needed because generateVpDynamicShadowsParams() is not declared
// virtual.
void GzTerrainMatGen::SM2Profile::ShaderHelperGLSL::generateVpHeader(
    const SM2Profile *_prof, const Ogre::Terrain *_terrain,
    TechniqueType _tt, Ogre::StringStream &_outStream)
{
  bool compression = false;

  _outStream << "#version " << glslVersion << "\n\n";

#if OGRE_VERSION_MAJOR >= 1 && OGRE_VERSION_MINOR >= 8
  compression = _terrain->_getUseVertexCompression() &&
                _tt != RENDER_COMPOSITE_MAP;

  if (compression)
  {
    // The parameter "in vec4 vertex;" is automatically bound by OGRE.
    // The parameter "in vec4 uv0'" is automatically bound by OGRE.
    _outStream << vpInStr << " vec4 vertex;\n"
               << vpInStr << " vec4 uv0;\n";
  }
  else
#endif
  {
    // The parameter "in vec4 vertex;" is automatically bound by OGRE.
    // The parameter "in vec4 uv0'" is automatically bound by OGRE.
    _outStream << vpInStr << " vec4 vertex;\n"
               << vpInStr << " vec4 uv0;\n";
  }

  if (_tt != RENDER_COMPOSITE_MAP)
    // The parameter "in vec4 uv1'" is automatically bound by OGRE.
    _outStream << vpInStr << " vec4 uv1;\n";

  _outStream <<
    "uniform mat4 worldMatrix;\n"
    "uniform mat4 viewProjMatrix;\n"
    "uniform vec2 lodMorph;\n";

  if (compression)
  {
    _outStream <<
      "uniform mat4  posIndexToObjectSpace;\n"
      "uniform float baseUVScale;\n";
  }


  // uv multipliers
  unsigned int maxLayers = _prof->getMaxLayers(_terrain);
  unsigned int numLayers = std::min(maxLayers,
      static_cast<unsigned int>(_terrain->getLayerCount()));

  unsigned int numUVMultipliers = (numLayers / 4);

  if (numLayers % 4)
    ++numUVMultipliers;

  for (unsigned int i = 0; i < numUVMultipliers; ++i)
    _outStream << "uniform vec4 uvMul" << i << ";\n";

  _outStream <<
    vpOutStr << " vec4 position;\n";

  unsigned int texCoordSet = 1;
  _outStream << vpOutStr << " vec4 uvMisc;\n";

  // layer UV's premultiplied, packed as xy/zw
  unsigned int numUVSets = numLayers / 2;

  if (numLayers % 2)
    ++numUVSets;

  if (_tt != LOW_LOD)
  {
    for (unsigned int i = 0; i < numUVSets; ++i)
    {
      _outStream << vpOutStr << " vec4 layerUV" << i << ";\n";
    }
  }

  if (_prof->getParent()->getDebugLevel() && _tt != RENDER_COMPOSITE_MAP)
  {
    _outStream << vpOutStr << " vec2 lodInfo;\n";
  }

  bool fog = _terrain->getSceneManager()->getFogMode() != Ogre::FOG_NONE &&
             _tt != RENDER_COMPOSITE_MAP;

  if (fog)
  {
    _outStream <<
      "uniform vec4 fogParams;\n"
      << vpOutStr << " float fogVal;\n";
  }

  if (_prof->isShadowingEnabled(_tt, _terrain))
  {
    texCoordSet = this->generateVpDynamicShadowsParams(texCoordSet, _prof,
        _terrain, _tt, _outStream);
  }

  // check we haven't exceeded texture coordinates
  if (texCoordSet > 8)
  {
    OGRE_EXCEPT(Ogre::Exception::ERR_INVALIDPARAMS,
        "Requested options require too many texture coordinate sets! "
        "Try reducing the number of layers.",
        __FUNCTION__);
  }

  _outStream << "void main()\n"
             << "{\n";

  if (compression)
  {
    _outStream
      << "  vec4 pos = posIndexToObjectSpace * "
      << "vec4(vertex.x, vertex.y, uv0.x, 1.0);\n"

      << "  vec2 uv = vec2(vertex.x * baseUVScale, 1.0 - "
      << "(vertex.y * baseUVScale));\n";
  }
  else
  {
    _outStream
      << "  vec4 pos = vertex;\n"
      << "  vec2 uv = vec2(uv0.x, uv0.y);\n";
  }

  _outStream << "  vec4 worldPos = worldMatrix * pos;\n";
  _outStream << "  position = pos;\n";

  if (_tt != RENDER_COMPOSITE_MAP)
  {
    // determine whether to apply the LOD morph to this vertex
    // we store the deltas against all vertices so we only want to apply
    // the morph to the ones which would disappear. The target LOD which is
    // being morphed to is stored in lodMorph.y, and the LOD at which
    // the vertex should be morphed is stored in uv.w. If we subtract
    // the former from the latter, and arrange to only morph if the
    // result is negative (it will only be -1 in fact, since after that
    // the vertex will never be indexed), we will achieve our aim.
    // sign(vertexLOD - targetLOD) == -1 is to morph
    _outStream <<
      "  float toMorph = -min(0.0, sign(uv1.y - lodMorph.y));\n";

    // this will either be 1 (morph) or 0 (don't morph)
    if (_prof->getParent()->getDebugLevel())
    {
      // x == LOD level (-1 since value is target level, we want to
      // display actual)
      _outStream << "lodInfo.x = (lodMorph.y - 1.0) / "
                 << _terrain->getNumLodLevels() << ";\n";

      // y == LOD morph
      _outStream << "lodInfo.y = toMorph * lodMorph.x;\n";
    }

    // morph
    switch (_terrain->getAlignment())
    {
      case Ogre::Terrain::ALIGN_X_Y:
        _outStream << "  worldPos.z += uv1.x * toMorph * lodMorph.x;\n";
        break;
      case Ogre::Terrain::ALIGN_X_Z:
        _outStream << "  worldPos.y += uv1.x * toMorph * lodMorph.x;\n";
        break;
      case Ogre::Terrain::ALIGN_Y_Z:
        _outStream << "  worldPos.x += uv1.x * toMorph * lodMorph.x;\n";
        break;
      default:
        gzerr << "Invalid alignment\n";
    };
  }

  // generate UVs
  if (_tt != LOW_LOD)
  {
    for (unsigned int i = 0; i < numUVSets; ++i)
    {
      unsigned int layer  =  i * 2;
      unsigned int uvMulIdx = layer / 4;

      _outStream << "  layerUV" << i << ".xy = " << " uv.xy * uvMul"
                 << uvMulIdx << "." << this->GetChannel(layer) << ";\n";
      _outStream << "  layerUV" << i << ".zw = " << " uv.xy * uvMul"
                 << uvMulIdx << "." << this->GetChannel(layer+1) << ";\n";
    }
  }
}

/////////////////////////////////////////////////
// This method is identical to
// TerrainMaterialGeneratorA::SM2Profile::ShaderHelperGLSL::generateVpFooter()
// but is needed because generateVpDynamicShadows() is not declared virtual.
void GzTerrainMatGen::SM2Profile::ShaderHelperGLSL::generateVpFooter(
    const SM2Profile *_prof, const Ogre::Terrain *_terrain,
    TechniqueType _tt, Ogre::StringStream &_outStream)
{
  _outStream << "  gl_Position = viewProjMatrix * worldPos;\n"
             << "  uvMisc.xy = uv.xy;\n";

  bool fog = _terrain->getSceneManager()->getFogMode() != Ogre::FOG_NONE &&
             _tt != RENDER_COMPOSITE_MAP;
  if (fog)
  {
    if (_terrain->getSceneManager()->getFogMode() == Ogre::FOG_LINEAR)
    {
      _outStream <<
        "  fogVal = clamp((oPos.z - fogParams.y) * fogParams.w, 0.0, 1.0);\n";
    }
    else
    {
      _outStream <<
        "  fogVal = 1 - clamp(1 / (exp(oPos.z * fogParams.x)), 0.0, 1.0);\n";
    }
  }

  if (_prof->isShadowingEnabled(_tt, _terrain))
    this->generateVpDynamicShadows(_prof, _terrain, _tt, _outStream);

  _outStream << "}\n";
}

/////////////////////////////////////////////////
void
GzTerrainMatGen::SM2Profile::ShaderHelperGLSL::generateVpDynamicShadows(
    const SM2Profile *_prof, const Ogre::Terrain * /*_terrain*/,
    TechniqueType /*_tt*/, Ogre::StringStream &_outStream)
{
  unsigned int numTextures = 1;

  if (_prof->getReceiveDynamicShadowsPSSM())
  {
    numTextures = _prof->getReceiveDynamicShadowsPSSM()->getSplitCount();
  }

  // Calculate the position of vertex in light space
  for (unsigned int i = 0; i < numTextures; ++i)
  {
    _outStream << "  lightSpacePos" << i << " = texViewProjMatrix"
               << i << " * worldPos;\n";

    // Don't linearize depth range: RTSS PSSM implementation uses
    // view-space depth
    // if (prof->getReceiveDynamicShadowsDepth())
    // {
    //   // make linear
    //   outStream << "lightSpacePos" << i << ".z = (lightSpacePos" << i
    //             << ".z - depthRange" << i << ".x) * depthRange" << i
    //             << ".w;\n";
    // }
  }

  if (_prof->getReceiveDynamicShadowsPSSM())
  {
    _outStream << "  // pass cam depth\n  uvMisc.z = gl_Position.z;\n";
  }
}

/////////////////////////////////////////////////
void GzTerrainMatGen::SM2Profile::ShaderHelperGLSL::defaultVpParams(
    const SM2Profile *_prof, const Ogre::Terrain *_terrain,
    TechniqueType _tt, const Ogre::HighLevelGpuProgramPtr &_prog)
{
  Ogre::GpuProgramParametersSharedPtr params = _prog->getDefaultParameters();
  params->setIgnoreMissingParams(true);

  params->setNamedAutoConstant("worldMatrix",
      Ogre::GpuProgramParameters::ACT_WORLD_MATRIX);

  params->setNamedAutoConstant("viewProjMatrix",
      Ogre::GpuProgramParameters::ACT_VIEWPROJ_MATRIX);

  params->setNamedAutoConstant("lodMorph",
      Ogre::GpuProgramParameters::ACT_CUSTOM,
      Ogre::Terrain::LOD_MORPH_CUSTOM_PARAM);

  params->setNamedAutoConstant("fogParams",
      Ogre::GpuProgramParameters::ACT_FOG_PARAMS);

  if (_prof->isShadowingEnabled(_tt, _terrain))
  {
    unsigned int numTextures = 1;
    if (_prof->getReceiveDynamicShadowsPSSM())
    {
      numTextures = _prof->getReceiveDynamicShadowsPSSM()->getSplitCount();
    }
    for (unsigned int i = 0; i < numTextures; ++i)
    {
      params->setNamedAutoConstant("texViewProjMatrix" +
          Ogre::StringConverter::toString(i),
          Ogre::GpuProgramParameters::ACT_TEXTURE_VIEWPROJ_MATRIX, i);

      // Don't add depth range params
      // if (prof->getReceiveDynamicShadowsDepth())
      // {
      //   params->setNamedAutoConstant("depthRange" +
      //       Ogre::StringConverter::toString(i),
      //       Ogre::GpuProgramParameters::ACT_SHADOW_SCENE_DEPTH_RANGE, i);
      // }
    }
  }

#if OGRE_VERSION_MAJOR >= 1 && OGRE_VERSION_MINOR >= 8
  if (_terrain->_getUseVertexCompression() && _tt != RENDER_COMPOSITE_MAP)
  {
    Ogre::Matrix4 posIndexToObjectSpace;
    _terrain->getPointTransform(&posIndexToObjectSpace);
    params->setNamedConstant("posIndexToObjectSpace", posIndexToObjectSpace);
  }
#endif
}

/////////////////////////////////////////////////
unsigned int GzTerrainMatGen::SM2Profile::ShaderHelperGLSL::
generateVpDynamicShadowsParams(unsigned int _texCoord, const SM2Profile *_prof,
    const Ogre::Terrain * /*_terrain*/, TechniqueType /*_tt*/,
    Ogre::StringStream &_outStream)
{
  // out semantics & params
  unsigned int numTextures = 1;

  if (_prof->getReceiveDynamicShadowsPSSM())
  {
    numTextures = _prof->getReceiveDynamicShadowsPSSM()->getSplitCount();
  }

  for (unsigned int i = 0; i < numTextures; ++i)
  {
    _outStream << vpOutStr << " vec4 lightSpacePos" << i << ";\n"
               << "uniform mat4 texViewProjMatrix" << i << ";\n";

    // Don't add depth range params
    // if (prof->getReceiveDynamicShadowsDepth())
    // {
    //   _outStream << ", uniform float4 depthRange" << i
    //             << " // x = min, y = max, z = range, w = 1/range\n";
    // }
  }

  return _texCoord;
}

/////////////////////////////////////////////////
void GzTerrainMatGen::SM2Profile::ShaderHelperGLSL::generateFpHeader(
    const SM2Profile *_prof, const Ogre::Terrain *_terrain,
    TechniqueType _tt, Ogre::StringStream &_outStream)
{
  _outStream << "#version " << glslVersion << "\n\n";

  _outStream <<
    "vec4 expand(vec4 v)\n"
    "{\n"
    "  return v * 2 - 1;\n"
    "}\n\n";

  _outStream <<
    "vec4 lit(float NdotL, float NdotH, float m)\n"
    "{\n"
    "  float specular = (NdotL > 0) ? pow(max(0.0, NdotH), m) : 0.0;\n"
    "  return vec4(1.0, max(0.0, NdotL), specular, 1.0);\n"
    "}\n";

  if (_prof->isShadowingEnabled(_tt, _terrain))
    this->generateFpDynamicShadowsHelpers(_prof, _terrain, _tt, _outStream);

  _outStream <<
    fpInStr << " vec4 position;\n";

  Ogre::uint texCoordSet = 1;
  _outStream << fpInStr << " vec4 uvMisc;\n";

  // UV's premultiplied, packed as xy/zw
  Ogre::uint maxLayers = _prof->getMaxLayers(_terrain);
  Ogre::uint numBlendTextures = std::min(
      _terrain->getBlendTextureCount(maxLayers),
      _terrain->getBlendTextureCount());
  Ogre::uint numLayers = std::min(maxLayers,
      static_cast<Ogre::uint>(_terrain->getLayerCount()));

  Ogre::uint numUVSets = numLayers / 2;

  if (numLayers % 2)
    ++numUVSets;

  if (_tt != LOW_LOD)
  {
    for (Ogre::uint i = 0; i < numUVSets; ++i)
    {
      _outStream <<
        fpInStr << " vec4 layerUV" << i << ";\n";
    }
  }

  if (_prof->getParent()->getDebugLevel() && _tt != RENDER_COMPOSITE_MAP)
  {
    _outStream << fpInStr << " vec2 lodInfo;\n";
  }

  bool fog = _terrain->getSceneManager()->getFogMode() != Ogre::FOG_NONE &&
             _tt != RENDER_COMPOSITE_MAP;

  if (fog)
  {
    _outStream <<
      "uniform vec3 fogColour;\n"
      << fpInStr << " float fogVal;\n";
  }

  Ogre::uint currentSamplerIdx = 0;

  _outStream <<
    // Only 1 light supported in this version
    // deferred shading profile / generator later, ok? :)
    "uniform vec3 ambient;\n"
    "uniform vec4 lightPosObjSpace;\n"
    "uniform vec3 lightDiffuseColour;\n"
    "uniform vec3 lightSpecularColour;\n"
    "uniform vec3 eyePosObjSpace;\n"
    // pack scale, bias and specular
    "uniform vec4 scaleBiasSpecular;\n";

  if (_tt == LOW_LOD)
  {
    // single composite map covers all the others below
    _outStream << "uniform sampler2D compositeMap;\n";
  }
  else
  {
    _outStream << "uniform sampler2D globalNormal;\n";

    if (_terrain->getGlobalColourMapEnabled() &&
        _prof->isGlobalColourMapEnabled())
    {
      _outStream << "uniform sampler2D globalColourMap;\n";
    }

    if (_prof->isLightmapEnabled())
    {
      _outStream << "uniform sampler2D lightMap;\n";
    }

    // Blend textures - sampler definitions
    for (Ogre::uint i = 0; i < numBlendTextures; ++i)
    {
      _outStream << "uniform sampler2D blendTex" << i << ";\n";
    }

    // Layer textures - sampler definitions & UV multipliers
    for (Ogre::uint i = 0; i < numLayers; ++i)
    {
      _outStream << "uniform sampler2D difftex" << i << ";\n";
      _outStream << "uniform sampler2D normtex" << i << ";\n";
    }
  }

  if (_prof->isShadowingEnabled(_tt, _terrain))
  {
    this->generateFpDynamicShadowsParams(&texCoordSet, &currentSamplerIdx,
        _prof, _terrain, _tt, _outStream);
  }

  // check we haven't exceeded samplers
  if (currentSamplerIdx > 16)
  {
    OGRE_EXCEPT(Ogre::Exception::ERR_INVALIDPARAMS,
        "Requested options require too many texture samplers! "
        "Try reducing the number of layers.", __FUNCTION__);
  }

  std::string outputColTypeStr = "vec4";
  if (glslVersion != "120")
  {
    _outStream << "out vec4 outputCol;\n";
    outputColTypeStr = "";
  }

  _outStream <<
    "void main()\n"
    "{\n"
    "  float shadow = 1.0;\n"
    "  vec2 uv = uvMisc.xy;\n"
    "  " << outputColTypeStr << " outputCol = vec4(0.0, 0.0, 0.0, 1.0);\n";

  if (_tt != LOW_LOD)
  {
    // global normal
    _outStream << "  vec3 normal = expand("
               << textureStr << "(globalNormal, uv)).xyz;\n";
  }

  _outStream <<
    "  vec3 lightDir =\n"
    "    lightPosObjSpace.xyz -  (position.xyz * lightPosObjSpace.w);\n"
    "  vec3 eyeDir = eyePosObjSpace - position.xyz;\n"

    // set up accumulation areas
    "  vec3 diffuse = vec3(0.0, 0.0, 0.0);\n"
    "  float specular = 0.0;\n";

  if (_tt == LOW_LOD)
  {
    // we just do a single calculation from composite map
    _outStream <<
      "  vec4 composite = " << textureStr << "(compositeMap, uv);\n"
      "  diffuse = composite.xyz;\n";
    // TODO - specular; we'll need normals for this!
  }
  else
  {
    // set up the blend values
    for (Ogre::uint i = 0; i < numBlendTextures; ++i)
    {
      _outStream << "  vec4 blendTexVal" << i
                 << " = " << textureStr << "(blendTex" << i << ", uv);\n";
    }

    if (_prof->isLayerNormalMappingEnabled())
    {
      // derive the tangent space basis
      // we do this in the pixel shader because we don't have per-vertex normals
      // because of the LOD, we use a normal map
      // tangent is always +x or -z in object space depending on alignment
      switch (_terrain->getAlignment())
      {
        case Ogre::Terrain::ALIGN_X_Y:
        case Ogre::Terrain::ALIGN_X_Z:
          _outStream << "  vec3 tangent = vec3(1.0, 0.0, 0.0);\n";
          break;
        case Ogre::Terrain::ALIGN_Y_Z:
          _outStream << "  vec3 tangent = vec3(0.0, 0.0, -1.0);\n";
          break;
        default:
          gzerr << "Invalid terrain alignment\n";
          break;
      };

      _outStream << "  vec3 binormal = normalize(cross(tangent, normal));\n";
      // note, now we need to re-cross to derive tangent again because it
      // wasn't orthonormal
      _outStream << "  tangent = normalize(cross(normal, binormal));\n";
      // derive final matrix
      /*_outStream << "  mat3 TBN = mat3(tangent.x, tangent.y, tangent.z,"
                                      "binormal.x, binormal.y, binormal.z,"
                                      "normal.x, normal.y, normal.z);\n";
                                      */

      // set up lighting result placeholders for interpolation
      _outStream << "  vec4 litRes, litResLayer;\n";
      _outStream << "  vec3 TSlightDir, TSeyeDir, TShalfAngle, TSnormal;\n";
      if (_prof->isLayerParallaxMappingEnabled())
        _outStream << "  float displacement;\n";
      // move
      _outStream << "  TSlightDir = normalize(vec3(dot(tangent, lightDir),"
                                              "dot(binormal, lightDir),"
                                              "dot(normal, lightDir)));\n";
      _outStream << "  TSeyeDir = normalize(vec3(dot(tangent, eyeDir),"
                                           "dot(binormal, eyeDir),"
                                           "dot(normal, eyeDir)));\n";
    }
    else
    {
      // simple per-pixel lighting with no normal mapping
      _outStream << "  lightDir = normalize(lightDir);\n";
      _outStream << "  eyeDir = normalize(eyeDir);\n";
      _outStream << "  vec3 halfAngle = normalize(lightDir + eyeDir);\n";

       _outStream << "  vec4 litRes = lit(dot(lightDir, normal), "
         "dot(halfAngle, normal), scaleBiasSpecular.z);\n";
    }
  }
}

/////////////////////////////////////////////////
void
GzTerrainMatGen::SM2Profile::ShaderHelperGLSL::generateFpDynamicShadowsParams(
    Ogre::uint *_texCoord, Ogre::uint *_sampler, const SM2Profile *_prof,
    const Ogre::Terrain * /*_terrain*/, TechniqueType _tt,
    Ogre::StringStream &_outStream)
{
  if (_tt == HIGH_LOD)
    this->mShadowSamplerStartHi = *_sampler;
  else if (_tt == LOW_LOD)
    this->mShadowSamplerStartLo = *_sampler;

  // in semantics & params
  Ogre::uint numTextures = 1;
  if (_prof->getReceiveDynamicShadowsPSSM())
  {
    numTextures = _prof->getReceiveDynamicShadowsPSSM()->getSplitCount();
    _outStream << "uniform vec4 pssmSplitPoints;\n";
  }

  for (Ogre::uint i = 0; i < numTextures; ++i)
  {
    _outStream << fpInStr <<
      " vec4 lightSpacePos" << i << ";\n" <<
      "uniform sampler2D shadowMap" << i << ";\n";

    *_sampler = *_sampler + 1;
    *_texCoord = *_texCoord + 1;

    if (_prof->getReceiveDynamicShadowsDepth())
    {
      _outStream <<
        "uniform float inverseShadowmapSize" << i << ";\n";
    }
  }
}

/////////////////////////////////////////////////
void GzTerrainMatGen::SM2Profile::ShaderHelperGLSL::generateFpLayer(
    const SM2Profile *_prof, const Ogre::Terrain * /*_terrain*/,
    TechniqueType _tt, Ogre::uint _layer,
    Ogre::StringStream &_outStream)
{
  Ogre::uint uvIdx = _layer / 2;
  Ogre::String uvChannels = (_layer % 2) ? ".zw" : ".xy";
  Ogre::uint blendIdx = (_layer-1) / 4;
  Ogre::String blendChannel = this->GetChannel(_layer-1);
  Ogre::String blendWeightStr = Ogre::String("blendTexVal") +
    Ogre::StringConverter::toString(blendIdx) + "." + blendChannel;

  // generate early-out conditional
  // Disable - causing some issues even when trying to force the use of texldd
  //   if (layer && prof->_isSM3Available())
  //   _outStream << "  if (" << blendWeightStr << " > 0.0003)\n  {\n";

  // generate UV
  _outStream << "  vec2 uv" << _layer << " = layerUV" << uvIdx
             << uvChannels << ";\n";

  // calculate lighting here if normal mapping
  if (_prof->isLayerNormalMappingEnabled())
  {
    if (_prof->isLayerParallaxMappingEnabled() && _tt != RENDER_COMPOSITE_MAP)
    {
      // modify UV - note we have to sample an extra time
      _outStream << "  displacement = " << textureStr << "(normtex" << _layer
                 << ", uv" << _layer << ").w\n"
        "   * scaleBiasSpecular.x + scaleBiasSpecular.y;\n";
      _outStream << "  uv" << _layer << " += TSeyeDir.xy * displacement;\n";
    }

    // access TS normal map
    _outStream << "  TSnormal = expand(" << textureStr << "(normtex"
               << _layer << ", uv" << _layer << ")).xyz;\n";
    _outStream << "  TShalfAngle = normalize(TSlightDir + TSeyeDir);\n";

    _outStream << "  litResLayer = lit(dot(TSlightDir, TSnormal), "
      "dot(TShalfAngle, TSnormal), scaleBiasSpecular.z);\n";

    if (!_layer)
      _outStream << "  litRes = litResLayer;\n";
    else
      _outStream << "  litRes = mix(litRes, litResLayer, "
                 << blendWeightStr << ");\n";
  }

  // sample diffuse texture
  _outStream << "  vec4 diffuseSpecTex" << _layer
    << " = " << textureStr << "(difftex" << _layer << ", uv" << _layer
    << ");\n";

  // apply to common
  if (!_layer)
  {
    _outStream << "  diffuse = diffuseSpecTex0.xyz;\n";
    if (_prof->isLayerSpecularMappingEnabled())
      _outStream << "  specular = diffuseSpecTex0.w;\n";
  }
  else
  {
     _outStream << "  diffuse = mix(diffuse, diffuseSpecTex" << _layer
                << ".xyz, " << blendWeightStr << ");\n";

    if (_prof->isLayerSpecularMappingEnabled())
    {
       _outStream << "  specular = mix(specular, diffuseSpecTex" << _layer
                  << ".w, " << blendWeightStr << ");\n";
    }
  }

  // End early-out
  // Disable - causing some issues even when trying to force the use of texldd
  //   if (layer && prof->_isSM3Available())
  //   _outStream << "  } // early-out blend value\n";
}

/////////////////////////////////////////////////
void GzTerrainMatGen::SM2Profile::ShaderHelperGLSL::generateFpFooter(
    const SM2Profile *_prof, const Ogre::Terrain *_terrain,
    TechniqueType _tt, Ogre::StringStream &_outStream)
{
  if (_tt == LOW_LOD)
  {
    if (_prof->isShadowingEnabled(_tt, _terrain))
    {
      this->generateFpDynamicShadows(_prof, _terrain, _tt, _outStream);
      _outStream << "  outputCol.xyz = diffuse * rtshadow;\n";
    }
    else
    {
      _outStream << "  outputCol.xyz = diffuse;\n";
    }
  }
  else
  {
    if (_terrain->getGlobalColourMapEnabled() &&
        _prof->isGlobalColourMapEnabled())
    {
      // sample colour map and apply to diffuse
      _outStream << "  diffuse *= " << textureStr
                 << "(globalColourMap, uv).xyz;\n";
    }

    if (_prof->isLightmapEnabled())
    {
      // sample lightmap
      _outStream << "  shadow = " << textureStr << "(lightMap, uv).x;\n";
    }

    if (_prof->isShadowingEnabled(_tt, _terrain))
    {
      this->generateFpDynamicShadows(_prof, _terrain, _tt, _outStream);
    }

    // diffuse lighting
    _outStream << "  outputCol.xyz += ambient * diffuse + litRes.y * "
                  "lightDiffuseColour * diffuse * shadow;\n";

    // specular default
    if (!_prof->isLayerSpecularMappingEnabled())
      _outStream << "  specular = 1.0;\n";

    if (_tt == RENDER_COMPOSITE_MAP)
    {
      // Lighting embedded in alpha
      _outStream << "  outputCol.w = shadow;\n";
    }
    else
    {
      // Apply specular
      _outStream << "  outputCol.xyz += litRes.z * lightSpecularColour * "
                    "specular * shadow;\n";

      if (_prof->getParent()->getDebugLevel())
      {
        _outStream << "  outputCol.xy += lodInfo.xy;\n";
      }
    }
  }

  bool fog = _terrain->getSceneManager()->getFogMode() != Ogre::FOG_NONE &&
             _tt != RENDER_COMPOSITE_MAP;
  if (fog)
  {
    _outStream << "  outputCol.xyz = mix(outputCol.xyz, fogColour, fogVal);\n";
  }

  if (glslVersion == "120")
    _outStream << "  gl_FragColor = outputCol;\n";

  // Final return
  _outStream << "\n}\n";
}

/////////////////////////////////////////////////
void
GzTerrainMatGen::SM2Profile::ShaderHelperGLSL::generateFpDynamicShadowsHelpers(
    const SM2Profile *_prof, const Ogre::Terrain * /*_terrain*/,
    TechniqueType /*_tt*/, Ogre::StringStream &_outStream)
{
  // TODO make filtering configurable
  _outStream <<
    "// Simple PCF\n"
    "// Number of samples in one dimension (square for total samples)\n"
    "#define NUM_SHADOW_SAMPLES_1D 2.0\n"
    "#define SHADOW_FILTER_SCALE 1.0\n"

    "#define SHADOW_SAMPLES NUM_SHADOW_SAMPLES_1D*NUM_SHADOW_SAMPLES_1D\n"

    "vec4 offsetSample(vec4 uv, vec2 offset, float invMapSize)\n"
    "{\n"
    "  return vec4(uv.xy + offset * invMapSize * uv.w, uv.z, uv.w);\n"
    "}\n";

  if (_prof->getReceiveDynamicShadowsDepth())
  {
    _outStream <<
      "float calcDepthShadow(sampler2D shadowMap, vec4 uv, "
      "float invShadowMapSize)\n"
      "{\n"
      "  // 4-sample PCF\n"
      "  float shadow = 0.0;\n"
      "  float offset = (NUM_SHADOW_SAMPLES_1D/2.0 - 0.5) *SHADOW_FILTER_SCALE;"
      "\n"
      "  for (float y = -offset; y <= offset; y += SHADOW_FILTER_SCALE)\n"
      "    for (float x = -offset; x <= offset; x += SHADOW_FILTER_SCALE)\n"
      "    {\n"
      "      vec4 newUV = offsetSample(uv, vec2(x, y), invShadowMapSize);\n"
      "      // manually project and assign derivatives\n"
      "      // to avoid gradient issues inside loops\n"
      "      newUV = newUV / newUV.w;\n";
      // The following line used to be:
      // "      float depth = tex2d(shadowMap, newUV.xy).x;\n"
    if (glslVersion == "120")
      _outStream <<
          "      float depth = texture2D(shadowMap, newUV.xy).x;\n";
    else
    {
      _outStream <<
          "      float depth = textureGrad(shadowMap, newUV.xy, "
          " vec2(1.0, 1.0), vec2(1.0, 1.0)).x;\n";
    }
    _outStream <<
      // "      if (depth >= 1.0 || depth >= uv.z)\n"
      "      if (depth >= 1.0 || depth >= newUV.z)\n"
      "        shadow += 1.0;\n"
      "    }\n"
      "  shadow /= (SHADOW_SAMPLES); \n"
      "  return shadow;\n"
      "}\n";
  }
  else
  {
    _outStream <<
      "float calcSimpleShadow(sampler2D shadowMap, vec4 shadowMapPos)\n"
      "{\n"
      "  return " << textureStr << "Proj(shadowMap, shadowMapPos).x;\n"
      "}\n";
  }

  if (_prof->getReceiveDynamicShadowsPSSM())
  {
    Ogre::uint numTextures =
      _prof->getReceiveDynamicShadowsPSSM()->getSplitCount();

    if (_prof->getReceiveDynamicShadowsDepth())
    {
      _outStream << "float calcPSSMDepthShadow(";
    }
    else
    {
      _outStream << "float calcPSSMSimpleShadow(";
    }

    _outStream << "\n  ";

    for (Ogre::uint i = 0; i < numTextures; ++i)
      _outStream << "sampler2D shadowMap" << i << ", ";

    _outStream << "\n  ";

    for (Ogre::uint i = 0; i < numTextures; ++i)
      _outStream << "vec4 lsPos" << i << ", ";

    if (_prof->getReceiveDynamicShadowsDepth())
    {
      _outStream << "\n  ";
      for (Ogre::uint i = 0; i < numTextures; ++i)
        _outStream << "float invShadowmapSize" << i << ", ";
    }

    _outStream << "\n"
      "  vec4 pssmSplitPoints, float camDepth)\n"
      "{\n"
      "  float shadow = 1.0;\n"
      "  // calculate shadow\n";

    for (Ogre::uint i = 0; i < numTextures; ++i)
    {
      if (!i)
      {
        _outStream << "  if (camDepth <= pssmSplitPoints."
          << this->GetChannel(i) << ")\n";
      }
      else if (i < numTextures-1)
      {
        _outStream << "  else if (camDepth <= pssmSplitPoints."
          << this->GetChannel(i) << ")\n";
      }
      else
        _outStream << "  else\n";

      _outStream << "  {\n";

      if (_prof->getReceiveDynamicShadowsDepth())
      {
        _outStream << "    shadow = calcDepthShadow(shadowMap" << i
          << ", lsPos" << i << ", invShadowmapSize" << i << ");\n";
      }
      else
      {
        _outStream << "    shadow = calcSimpleShadow(shadowMap" << i
          << ", lsPos" << i << ");\n";
      }
      _outStream << "  }\n";
    }

    _outStream << "  return shadow;\n"
                  "}\n\n\n";
  }
}

/////////////////////////////////////////////////
void GzTerrainMatGen::SM2Profile::ShaderHelperGLSL::generateFpDynamicShadows(
    const SM2Profile *_prof, const Ogre::Terrain * /*_terrain*/,
    TechniqueType /*_tt*/, Ogre::StringStream &_outStream)
{
  if (_prof->getReceiveDynamicShadowsPSSM())
  {
    Ogre::uint numTextures =
      _prof->getReceiveDynamicShadowsPSSM()->getSplitCount();

    _outStream << "  float camDepth = uvMisc.z;\n";

    if (_prof->getReceiveDynamicShadowsDepth())
    {
      _outStream << "  float rtshadow = calcPSSMDepthShadow(";
    }
    else
    {
      _outStream << "  float rtshadow = calcPSSMSimpleShadow(";
    }

    for (Ogre::uint i = 0; i < numTextures; ++i)
      _outStream << "shadowMap" << i << ", ";

    _outStream << "\n    ";

    for (Ogre::uint i = 0; i < numTextures; ++i)
      _outStream << "lightSpacePos" << i << ", ";

    if (_prof->getReceiveDynamicShadowsDepth())
    {
      _outStream << "\n    ";

      for (Ogre::uint i = 0; i < numTextures; ++i)
        _outStream << "inverseShadowmapSize" << i << ", ";
    }
    _outStream << "\n" <<
      "    pssmSplitPoints, camDepth);\n";
  }
  else
  {
    if (_prof->getReceiveDynamicShadowsDepth())
    {
      _outStream <<
        "  float rtshadow = calcDepthShadow(shadowMap0, lightSpacePos0, "
        "inverseShadowmapSize0);";
    }
    else
    {
      _outStream <<
        "  float rtshadow = calcSimpleShadow(shadowMap0, lightSpacePos0);";
    }
  }

  _outStream << "  shadow = rtshadow;//min(shadow, rtshadow);\n";
}

/////////////////////////////////////////////////
void
GzTerrainMatGen::SM2Profile::ShaderHelperGLSL::generateFragmentProgramSource(
    const SM2Profile *_prof, const Ogre::Terrain *_terrain,
    TechniqueType _tt, Ogre::StringStream &_outStream)
{
  this->generateFpHeader(_prof, _terrain, _tt, _outStream);

  if (_tt != LOW_LOD)
  {
    Ogre::uint maxLayers = _prof->getMaxLayers(_terrain);
    Ogre::uint numLayers = std::min(maxLayers,
        static_cast<Ogre::uint>(_terrain->getLayerCount()));

    for (Ogre::uint i = 0; i < numLayers; ++i)
      this->generateFpLayer(_prof, _terrain, _tt, i, _outStream);
  }

  this->generateFpFooter(_prof, _terrain, _tt, _outStream);
}

/////////////////////////////////////////////////
void GzTerrainMatGen::SM2Profile::ShaderHelperGLSL::updateVpParams(
    const SM2Profile *_prof, const Ogre::Terrain *_terrain,
#if OGRE_VERSION_MAJOR >= 1 && OGRE_VERSION_MINOR >= 8
    TechniqueType _tt,
#else
    TechniqueType /*_tt*/,
#endif
    const Ogre::GpuProgramParametersSharedPtr &_params)
{
  _params->setIgnoreMissingParams(true);
  Ogre::uint maxLayers = _prof->getMaxLayers(_terrain);
  Ogre::uint numLayers = std::min(maxLayers,
      static_cast<Ogre::uint>(_terrain->getLayerCount()));

  Ogre::uint numUVMul = numLayers / 4;

  if (numLayers % 4)
    ++numUVMul;

  for (Ogre::uint i = 0; i < numUVMul; ++i)
  {
    Ogre::Vector4 uvMul(
        _terrain->getLayerUVMultiplier(i * 4),
        _terrain->getLayerUVMultiplier(i * 4 + 1),
        _terrain->getLayerUVMultiplier(i * 4 + 2),
        _terrain->getLayerUVMultiplier(i * 4 + 3));
    _params->setNamedConstant("uvMul" +
        Ogre::StringConverter::toString(i), uvMul);
  }

#if OGRE_VERSION_MAJOR >= 1 && OGRE_VERSION_MINOR >= 8
  if (_terrain->_getUseVertexCompression() && _tt != RENDER_COMPOSITE_MAP)
  {
    Ogre::Real baseUVScale = 1.0f / (_terrain->getSize() - 1);
    _params->setNamedConstant("baseUVScale", baseUVScale);
  }
#endif
}

/////////////////////////////////////////////////
Ogre::String GzTerrainMatGen::SM2Profile::ShaderHelperGLSL::GetChannel(
Ogre::uint _idx)
{
  Ogre::uint rem = _idx % 4;
  switch (rem)
  {
    case 0:
    default:
      return "x";
    case 1:
      return "y";
    case 2:
      return "z";
    case 3:
      return "w";
  };
}

/////////////////////////////////////////////////
/////////////////////////////////////////////////
// CG Shader helper
/////////////////////////////////////////////////
/////////////////////////////////////////////////

/////////////////////////////////////////////////
Ogre::HighLevelGpuProgramPtr
GzTerrainMatGen::SM2Profile::ShaderHelperCg::generateVertexProgram(
    const SM2Profile *_prof, const Ogre::Terrain *_terrain,
    TechniqueType _tt)
{
  Ogre::HighLevelGpuProgramPtr ret =
    this->createVertexProgram(_prof, _terrain, _tt);

  Ogre::StringStream sourceStr;
  this->generateVertexProgramSource(_prof, _terrain, _tt, sourceStr);

  ret->setSource(sourceStr.str());
  ret->load();
  this->defaultVpParams(_prof, _terrain, _tt, ret);

  return ret;
}

/////////////////////////////////////////////////
void GzTerrainMatGen::SM2Profile::ShaderHelperCg::defaultVpParams(
    const SM2Profile *_prof, const Ogre::Terrain *_terrain,
    TechniqueType _tt, const Ogre::HighLevelGpuProgramPtr &_prog)
{
  Ogre::GpuProgramParametersSharedPtr params = _prog->getDefaultParameters();
  params->setIgnoreMissingParams(true);

  params->setNamedAutoConstant("worldMatrix",
      Ogre::GpuProgramParameters::ACT_WORLD_MATRIX);

  params->setNamedAutoConstant("viewProjMatrix",
      Ogre::GpuProgramParameters::ACT_VIEWPROJ_MATRIX);

  params->setNamedAutoConstant("lodMorph",
      Ogre::GpuProgramParameters::ACT_CUSTOM,
      Ogre::Terrain::LOD_MORPH_CUSTOM_PARAM);

  params->setNamedAutoConstant("fogParams",
      Ogre::GpuProgramParameters::ACT_FOG_PARAMS);

  if (_prof->isShadowingEnabled(_tt, _terrain))
  {
    unsigned int numTextures = 1;
    if (_prof->getReceiveDynamicShadowsPSSM())
    {
      numTextures = _prof->getReceiveDynamicShadowsPSSM()->getSplitCount();
    }
    for (unsigned int i = 0; i < numTextures; ++i)
    {
      params->setNamedAutoConstant("texViewProjMatrix" +
          Ogre::StringConverter::toString(i),
          Ogre::GpuProgramParameters::ACT_TEXTURE_VIEWPROJ_MATRIX, i);

      // Don't add depth range params
      // if (prof->getReceiveDynamicShadowsDepth())
      // {
      //   params->setNamedAutoConstant("depthRange" +
      //       Ogre::StringConverter::toString(i),
      //       Ogre::GpuProgramParameters::ACT_SHADOW_SCENE_DEPTH_RANGE, i);
      // }
    }
  }

#if OGRE_VERSION_MAJOR >= 1 && OGRE_VERSION_MINOR >= 8
  if (_terrain->_getUseVertexCompression() && _tt != RENDER_COMPOSITE_MAP)
  {
    Ogre::Matrix4 posIndexToObjectSpace;
    _terrain->getPointTransform(&posIndexToObjectSpace);
    params->setNamedConstant("posIndexToObjectSpace", posIndexToObjectSpace);
  }
#endif
}

/////////////////////////////////////////////////
void
GzTerrainMatGen::SM2Profile::ShaderHelperCg::generateVpDynamicShadows(
    const SM2Profile *_prof, const Ogre::Terrain * /*_terrain*/,
    TechniqueType /*_tt*/, Ogre::StringStream &_outStream)
{
  unsigned int numTextures = 1;

  if (_prof->getReceiveDynamicShadowsPSSM())
  {
    numTextures = _prof->getReceiveDynamicShadowsPSSM()->getSplitCount();
  }

  // Calculate the position of vertex in light space
  for (unsigned int i = 0; i < numTextures; ++i)
  {
    _outStream << "  oLightSpacePos" << i << " = mul(texViewProjMatrix"
               << i << ", worldPos);\n";

    // Don't linearize depth range: RTSS PSSM implementation uses
    // view-space depth
    // if (prof->getReceiveDynamicShadowsDepth())
    // {
    //   // make linear
    //   outStream << "oLightSpacePos" << i << ".z = (oLightSpacePos" << i
    //             << ".z - depthRange" << i << ".x) * depthRange" << i
    //             << ".w;\n";
    // }
  }

  if (_prof->getReceiveDynamicShadowsPSSM())
  {
    _outStream << "  // pass cam depth\n   oUVMisc.z = oPos.z;\n";
  }
}

/////////////////////////////////////////////////
unsigned int GzTerrainMatGen::SM2Profile::ShaderHelperCg::
generateVpDynamicShadowsParams(unsigned int _texCoord, const SM2Profile *_prof,
    const Ogre::Terrain * /*_terrain*/, TechniqueType /*_tt*/,
    Ogre::StringStream &_outStream)
{
  // out semantics & params
  unsigned int numTextures = 1;

  if (_prof->getReceiveDynamicShadowsPSSM())
  {
    numTextures = _prof->getReceiveDynamicShadowsPSSM()->getSplitCount();
  }

  for (unsigned int i = 0; i < numTextures; ++i)
  {
    _outStream << ", out float4 oLightSpacePos" << i
               << " : TEXCOORD" << _texCoord++ << "\n"
               << ", uniform float4x4 texViewProjMatrix" << i << "\n";

    // Don't add depth range params
    // if (prof->getReceiveDynamicShadowsDepth())
    // {
    //   _outStream << ", uniform float4 depthRange" << i
    //             << " // x = min, y = max, z = range, w = 1/range\n";
    // }
  }

  return _texCoord;
}

/////////////////////////////////////////////////
// This method is identical to
// TerrainMaterialGeneratorA::SM2Profile::ShaderHelperCg::generateVpHeader()
// but is needed because generateVpDynamicShadowsParams() is not declared
// virtual.
void GzTerrainMatGen::SM2Profile::ShaderHelperCg::generateVpHeader(
    const SM2Profile *_prof, const Ogre::Terrain *_terrain,
    TechniqueType _tt, Ogre::StringStream &_outStream)
{
  _outStream << "void main_vp(\n";

  bool compression = false;

#if OGRE_VERSION_MAJOR >= 1 && OGRE_VERSION_MINOR >= 8
  compression = _terrain->_getUseVertexCompression() &&
                _tt != RENDER_COMPOSITE_MAP;
#endif

  if (compression)
  {
    _outStream << "float2 posIndex : POSITION,\nfloat height  : TEXCOORD0,\n";
  }
  else
  {
    _outStream << "float4 pos : POSITION,\nfloat2 uv  : TEXCOORD0,\n";
  }

  if (_tt != RENDER_COMPOSITE_MAP)
    _outStream << "float2 delta  : TEXCOORD1,\n";

  _outStream <<
    "uniform float4x4 worldMatrix,\n"
    "uniform float4x4 viewProjMatrix,\n"
    "uniform float2   lodMorph,\n";

  if (compression)
  {
    _outStream <<
      "uniform float4x4   posIndexToObjectSpace,\n"
      "uniform float    baseUVScale,\n";
  }

  // uv multipliers
  Ogre::uint maxLayers = _prof->getMaxLayers(_terrain);
  Ogre::uint numLayers = std::min(maxLayers,
      static_cast<unsigned int>(_terrain->getLayerCount()));

  unsigned int numUVMultipliers = (numLayers / 4);

  if (numLayers % 4)
    ++numUVMultipliers;

  for (unsigned int i = 0; i < numUVMultipliers; ++i)
    _outStream << "uniform float4 uvMul" << i << ",\n";

  _outStream <<
    "out float4 oPos : POSITION,\n"
    "out float4 oPosObj : TEXCOORD0\n";

  unsigned int texCoordSet = 1;
  _outStream << ", out float4 oUVMisc : TEXCOORD" << texCoordSet++
            << " // xy = uv, z = camDepth\n";

  // layer UV's premultiplied, packed as xy/zw
  unsigned int numUVSets = numLayers / 2;

  if (numLayers % 2)
    ++numUVSets;

  if (_tt != LOW_LOD)
  {
    for (unsigned int i = 0; i < numUVSets; ++i)
    {
      _outStream << ", out float4 oUV" << i
                << " : TEXCOORD" << texCoordSet++ << "\n";
    }
  }

  if (_prof->getParent()->getDebugLevel() && _tt != RENDER_COMPOSITE_MAP)
  {
    _outStream << ", out float2 lodInfo : TEXCOORD" << texCoordSet++ << "\n";
  }

  bool fog = _terrain->getSceneManager()->getFogMode() != Ogre::FOG_NONE &&
             _tt != RENDER_COMPOSITE_MAP;

  if (fog)
  {
    _outStream <<
      ", uniform float4 fogParams\n"
      ", out float fogVal : COLOR\n";
  }

  if (_prof->isShadowingEnabled(_tt, _terrain))
  {
    texCoordSet = generateVpDynamicShadowsParams(texCoordSet, _prof,
        _terrain, _tt, _outStream);
  }

  // check we haven't exceeded texture coordinates
  if (texCoordSet > 8)
  {
    OGRE_EXCEPT(Ogre::Exception::ERR_INVALIDPARAMS,
        "Requested options require too many texture coordinate sets! "
        "Try reducing the number of layers.",
        __FUNCTION__);
  }

  _outStream <<
    ")\n"
    "{\n";

  if (compression)
  {
    _outStream << "  float4 pos;\n"
      << "  pos = mul(posIndexToObjectSpace, float4(posIndex, height, 1));\n"
      << "  float2 uv = float2(posIndex.x * baseUVScale, 1.0 - "
      << "(posIndex.y * baseUVScale));\n";
  }

  _outStream <<
    "  float4 worldPos = mul(worldMatrix, pos);\n"
    "  oPosObj = pos;\n";

  if (_tt != RENDER_COMPOSITE_MAP)
  {
    // determine whether to apply the LOD morph to this vertex
    // we store the deltas against all vertices so we only want to apply
    // the morph to the ones which would disappear. The target LOD which is
    // being morphed to is stored in lodMorph.y, and the LOD at which
    // the vertex should be morphed is stored in uv.w. If we subtract
    // the former from the latter, and arrange to only morph if the
    // result is negative (it will only be -1 in fact, since after that
    // the vertex will never be indexed), we will achieve our aim.
    // sign(vertexLOD - targetLOD) == -1 is to morph
    _outStream <<
      "  float toMorph = -min(0, sign(delta.y - lodMorph.y));\n";

    // this will either be 1 (morph) or 0 (don't morph)
    if (_prof->getParent()->getDebugLevel())
    {
      // x == LOD level (-1 since value is target level, we want to
      // display actual)
      _outStream << "lodInfo.x = (lodMorph.y - 1) / "
                 << _terrain->getNumLodLevels() << ";\n";

      // y == LOD morph
      _outStream << "lodInfo.y = toMorph * lodMorph.x;\n";
    }

    // morph
    switch (_terrain->getAlignment())
    {
      case Ogre::Terrain::ALIGN_X_Y:
        _outStream << "  worldPos.z += delta.x * toMorph * lodMorph.x;\n";
        break;
      case Ogre::Terrain::ALIGN_X_Z:
        _outStream << "  worldPos.y += delta.x * toMorph * lodMorph.x;\n";
        break;
      case Ogre::Terrain::ALIGN_Y_Z:
        _outStream << "  worldPos.x += delta.x * toMorph * lodMorph.x;\n";
        break;
      default:
        gzerr << "Invalid alignment\n";
    };
  }

  // generate UVs
  if (_tt != LOW_LOD)
  {
    for (unsigned int i = 0; i < numUVSets; ++i)
    {
      unsigned int layer  =  i * 2;
      unsigned int uvMulIdx = layer / 4;

      _outStream << "  oUV" << i << ".xy = " << " uv.xy * uvMul"
                 << uvMulIdx << "." << getChannel(layer) << ";\n";
      _outStream << "  oUV" << i << ".zw = " << " uv.xy * uvMul"
                 << uvMulIdx << "." << getChannel(layer+1) << ";\n";
    }
  }
}

/////////////////////////////////////////////////
// This method is identical to
// TerrainMaterialGeneratorA::SM2Profile::ShaderHelperCg::generateVpFooter()
// but is needed because generateVpDynamicShadows() is not declared virtual.
void GzTerrainMatGen::SM2Profile::ShaderHelperCg::generateVpFooter(
    const SM2Profile *_prof, const Ogre::Terrain *_terrain,
    TechniqueType _tt, Ogre::StringStream &_outStream)
{
  _outStream << "  oPos = mul(viewProjMatrix, worldPos);\n"
             << "  oUVMisc.xy = uv.xy;\n";

  bool fog = _terrain->getSceneManager()->getFogMode() != Ogre::FOG_NONE &&
             _tt != RENDER_COMPOSITE_MAP;
  if (fog)
  {
    if (_terrain->getSceneManager()->getFogMode() == Ogre::FOG_LINEAR)
    {
      _outStream <<
        "  fogVal = saturate((oPos.z - fogParams.y) * fogParams.w);\n";
    }
    else
    {
      _outStream <<
        "  fogVal = 1 - saturate(1 / (exp(oPos.z * fogParams.x)));\n";
    }
  }

  if (_prof->isShadowingEnabled(_tt, _terrain))
    this->generateVpDynamicShadows(_prof, _terrain, _tt, _outStream);

  _outStream << "}\n";
}

/////////////////////////////////////////////////
void GzTerrainMatGen::SM2Profile::ShaderHelperCg::
generateVertexProgramSource(const SM2Profile *_prof,
    const Ogre::Terrain* _terrain, TechniqueType _tt,
    Ogre::StringStream &_outStream)
{
  this->generateVpHeader(_prof, _terrain, _tt, _outStream);

  if (_tt != LOW_LOD)
  {
    unsigned int maxLayers = _prof->getMaxLayers(_terrain);
    unsigned int numLayers = std::min(maxLayers,
        static_cast<unsigned int>(_terrain->getLayerCount()));

    for (unsigned int i = 0; i < numLayers; ++i)
      this->generateVpLayer(_prof, _terrain, _tt, i, _outStream);
  }

  this->generateVpFooter(_prof, _terrain, _tt, _outStream);
}

/////////////////////////////////////////////////
Ogre::HighLevelGpuProgramPtr
GzTerrainMatGen::SM2Profile::ShaderHelperCg::generateFragmentProgram(
    const SM2Profile *_prof, const Ogre::Terrain *_terrain, TechniqueType _tt)
{
  Ogre::HighLevelGpuProgramPtr ret = this->createFragmentProgram(_prof,
      _terrain, _tt);

  Ogre::StringStream sourceStr;

  this->generateFragmentProgramSource(_prof, _terrain, _tt, sourceStr);

  ret->setSource(sourceStr.str());

  ret->load();

  this->defaultFpParams(_prof, _terrain, _tt, ret);

  return ret;
}

/////////////////////////////////////////////////
/////////////////////////////////////////////////
// TerrainMaterial
/////////////////////////////////////////////////
/////////////////////////////////////////////////

//////////////////////////////////////////////////
TerrainMaterial::TerrainMaterial(const std::string &_materialname)
{
  this->materialName = _materialname;
  this->mProfiles.push_back(OGRE_NEW Profile(this, "OgreMaterial",
      "Profile for rendering Ogre standard material"));
  this->setActiveProfile("OgreMaterial");
}

//////////////////////////////////////////////////
void TerrainMaterial::setMaterialByName(const std::string &_materialname)
{
  this->materialName = _materialname;
}

//////////////////////////////////////////////////
void TerrainMaterial::setGridSize(const unsigned int _size)
{
  if (_size == 0)
  {
    gzerr << "Unable to set a grid size of zero" << std::endl;
    return;
  }

  this->gridSize = _size;
}

//////////////////////////////////////////////////
TerrainMaterial::Profile::Profile(Ogre::TerrainMaterialGenerator *_parent,
    const Ogre::String &_name, const Ogre::String &_desc)
    : Ogre::TerrainMaterialGenerator::Profile(_parent, _name, _desc)
{
}

//////////////////////////////////////////////////
TerrainMaterial::Profile::~Profile()
{
}

//////////////////////////////////////////////////
bool TerrainMaterial::Profile::isVertexCompressionSupported() const
{
  return false;
}

//////////////////////////////////////////////////
Ogre::MaterialPtr TerrainMaterial::Profile::generate(
    const Ogre::Terrain *_terrain)
{
  const Ogre::String& matName = _terrain->getMaterialName();

  Ogre::MaterialPtr mat =
      Ogre::MaterialManager::getSingleton().getByName(matName);
  if (!mat.isNull())
      Ogre::MaterialManager::getSingleton().remove(matName);

  TerrainMaterial *parent =
      dynamic_cast<TerrainMaterial *>(getParent());

  // Set Ogre material
  mat = Ogre::MaterialManager::getSingleton().getByName(parent->materialName);

  // clone the material
  mat = mat->clone(matName);
  if (!mat->isLoaded())
    mat->load();

  // size of grid in one direction
  unsigned int gridWidth =
      static_cast<unsigned int>(std::sqrt(parent->gridSize));
  // factor to be applied to uv transformation: scale and translation
  double factor = 1.0 / gridWidth;
  // static counter to keep track which terrain slot we are currently in
  static int gridCount = 0;

  for (unsigned int i = 0; i < mat->getNumTechniques(); ++i)
  {
    Ogre::Technique *tech = mat->getTechnique(i);
    for (unsigned int j = 0; j < tech->getNumPasses(); ++j)
    {
      Ogre::Pass *pass = tech->getPass(j);

      // check if there is a fragment shader
      if (!pass->hasFragmentProgram())
        continue;

      Ogre::GpuProgramParametersSharedPtr params =
          pass->getFragmentProgramParameters();
      if (params.isNull())
        continue;

      // set up shadow split points in a way that is consistent with the
      // default ogre terrain material generator

      if (params->_findNamedConstantDefinition("pssmSplitPoints"))
      {
        Ogre::PSSMShadowCameraSetup* pssm =
            RTShaderSystem::Instance()->GetPSSMShadowCameraSetup();
        unsigned int numTextures =
            static_cast<unsigned int>(pssm->getSplitCount());
        Ogre::Vector4 splitPoints;
        const Ogre::PSSMShadowCameraSetup::SplitPointList& splitPointList =
            pssm->getSplitPoints();
        // populate from split point 1 not 0, and include shadowFarDistance
        for (unsigned int t = 0u; t < numTextures; ++t)
          splitPoints[t] = splitPointList[t+1];
        params->setNamedConstant("pssmSplitPoints", splitPoints);
      }

      if (params->_findNamedConstantDefinition("uvTransform"))
      {
        // set up uv transform
        double xTrans = static_cast<int>(gridCount / gridWidth) * factor;
        double yTrans = (gridWidth - 1 - (gridCount % gridWidth)) * factor;
        // explicitly set all matrix elements to avoid uninitialized values
        Ogre::Matrix4 uvTransform(factor, 0.0, 0.0, xTrans,
                                  0.0, factor, 0.0, yTrans,
                                  0.0, 0.0, 1.0, 0.0,
                                  0.0, 0.0, 0.0, 1.0);
        params->setNamedConstant("uvTransform", uvTransform);
      }
    }
  }
  gridCount++;

  // Get default pass
  Ogre::Pass *p = mat->getTechnique(0)->getPass(0);

  // Add terrain's global normalmap to renderpass so the
  // fragment program can find it.
  Ogre::TextureUnitState *tu = p->createTextureUnitState(matName+"/nm");

  Ogre::TexturePtr nmtx = _terrain->getTerrainNormalMap();
  tu->_setTexturePtr(nmtx);

  return mat;
}

//////////////////////////////////////////////////
Ogre::MaterialPtr TerrainMaterial::Profile::generateForCompositeMap(
    const Ogre::Terrain *_terrain)
{
  return _terrain->_getCompositeMapMaterial();
}

//////////////////////////////////////////////////
void TerrainMaterial::Profile::setLightmapEnabled(bool /*_enabled*/)
{
}

//////////////////////////////////////////////////
Ogre::uint8 TerrainMaterial::Profile::getMaxLayers(
    const Ogre::Terrain */*_terrain*/) const
{
  return 0;
}

//////////////////////////////////////////////////
void TerrainMaterial::Profile::updateParams(const Ogre::MaterialPtr &/*_mat*/,
    const Ogre::Terrain */*_terrain*/)
{
}

//////////////////////////////////////////////////
void TerrainMaterial::Profile::updateParamsForCompositeMap(
    const Ogre::MaterialPtr &/*_mat*/, const Ogre::Terrain */*_terrain*/)
{
}

//////////////////////////////////////////////////
void TerrainMaterial::Profile::requestOptions(Ogre::Terrain *_terrain)
{
  _terrain->_setMorphRequired(true);
  // enable global normal map
  _terrain->_setNormalMapRequired(true);
  _terrain->_setLightMapRequired(false);
  _terrain->_setCompositeMapRequired(false);
}
