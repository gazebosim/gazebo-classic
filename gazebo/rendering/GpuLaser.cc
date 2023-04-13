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

#include <algorithm>
#include <sstream>
#include <utility>

#include <ignition/common/Profiler.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/math/Vector3.hh>

#ifndef _WIN32
  #include <dirent.h>
#else
  #include "gazebo/common/win_dirent.h"
#endif

#include "gazebo/rendering/ogre_gazebo.h"

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"

#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/GpuLaser.hh"
#include "gazebo/rendering/GpuLaserPrivate.hh"

using namespace gazebo;
using namespace rendering;

//////////////////////////////////////////////////
GpuLaser::GpuLaser(const std::string &_namePrefix, ScenePtr _scene,
                   const bool _autoRender)
  : Camera(_namePrefix, std::move(_scene), _autoRender)
  , horzHalfAngle {0.0}
  , vertHalfAngle {0.0}
  , rayCountRatio {0.0}
  , hfov {0.0}
  , vfov {0.0}
  , chfov {0.0}
  , cvfov {0.0}
  , nearClip {0.0}
  , farClip {0.0}
  , isHorizontal {false}
  , isSampleSensor {false}
  , sampleSize {0}
  , sampleOffset {0}
  , dataPtr {std::make_unique<GpuLaserPrivate>()}
{
  this->dataPtr->material = nullptr;
  this->dataPtr->horizontalRangeCount = 0;
  this->dataPtr->verticalRangeCount = 0;
}

//////////////////////////////////////////////////
GpuLaser::~GpuLaser()
{
  this->Fini();
}

//////////////////////////////////////////////////
void GpuLaser::Load(sdf::ElementPtr _sdf)
{
  Camera::Load(_sdf);
}

//////////////////////////////////////////////////
void GpuLaser::Load()
{
  Camera::Load();
}

//////////////////////////////////////////////////
void GpuLaser::Init()
{
  Camera::Init();
}

//////////////////////////////////////////////////
void GpuLaser::Fini()
{
  for (const auto& [cubeFaceId, cubeFace] : this->dataPtr->cubeMapFaces)
  {
    if (!cubeFace.texture.isNull())
    {
      Ogre::TextureManager::getSingleton().remove(
          cubeFace.texture->getName());
    }
  }

  this->dataPtr->laserScan.clear();

  Camera::Fini();
}

//////////////////////////////////////////////////
void GpuLaser::CreateLaserTexture(const std::string &_textureName, const Ogre::PixelFormat _pixel_format)
{
  unsigned int cubeFaceTexIndex = 0;
  for (auto& [cubeFaceId, cubeFace] : this->dataPtr->cubeMapFaces)
  {
    std::stringstream texName;
    texName << _textureName << "_cube_face_texture_" << cubeFaceTexIndex;

    cubeFace.texture = Ogre::TextureManager::getSingleton().createManual(
        texName.str(), "General", Ogre::TEX_TYPE_2D,
        this->ImageWidth(), this->ImageHeight(), 0,
        _pixel_format, Ogre::TU_RENDERTARGET);

    this->SetUpRenderTarget(cubeFace);

    cubeFaceTexIndex++;
  }

  this->dataPtr->material =
      static_cast<Ogre::Material *>(Ogre::MaterialManager::getSingleton()
                                        .getByName("Gazebo/LaserScan")
                                        .get());
  GZ_ASSERT(this->dataPtr->material, "Couldn't find material Gazebo/LaserScan");

  this->dataPtr->material->load();
  this->dataPtr->material->setCullingMode(Ogre::CULL_NONE);
}

//////////////////////////////////////////////////
void GpuLaser::PostRender()
{
  for (auto& [cubeFaceId, cubeFace] : this->dataPtr->cubeMapFaces)
  {
    cubeFace.texture->getBuffer()->getRenderTarget()->swapBuffers();
  }

  unsigned int numberOfChannels;
  if (isSampleSensor)
  {
    numberOfChannels = 1;
  }
  else
  {
    numberOfChannels = 3;
  }

  IGN_PROFILE_BEGIN("blitToMemory");
  // Blit the depth buffer if needed
  if (this->newData && this->captureData)
  {
    const unsigned int imageWidth = this->ImageWidth();
    const unsigned int imageHeight = this->ImageHeight();

    size_t laserBufferSize;
    if (isSampleSensor)
    {
      laserBufferSize = this->sampleSize * numberOfChannels * 3;
    }
    else
    {
      laserBufferSize = this->dataPtr->horizontalRangeCount * this->dataPtr->verticalRangeCount * numberOfChannels;
    }

    const size_t depthImageBufferSize = imageWidth * imageHeight * numberOfChannels;
    if (this->dataPtr->laserBuffer.empty())
    {
      this->dataPtr->laserBuffer.resize(laserBufferSize);
    }

    for (auto& [cubeFaceId, cubeFace] : this->dataPtr->cubeMapFaces)
    {
      if (isSampleSensor)
      {
        cubeFace.depthImgFloat16.resize(depthImageBufferSize);
        Ogre::PixelBox dstBox(imageWidth, imageHeight, 1, Ogre::PF_FLOAT16_R, cubeFace.depthImgFloat16.data());
        cubeFace.texture->getBuffer()->blitToMemory(dstBox);
      }
      else
      {
        cubeFace.depthImgFloat32.resize(depthImageBufferSize);
        Ogre::PixelBox dstBox(imageWidth, imageHeight, 1, Ogre::PF_FLOAT32_RGB, cubeFace.depthImgFloat32.data());
        cubeFace.texture->getBuffer()->blitToMemory(dstBox);
      }
    }
    IGN_PROFILE_END();
    if (!isSampleSensor)
    {
      // read ranges
      GZ_ASSERT(this->dataPtr->horizontalRangeCount == this->dataPtr->mapping.size(), "cube face mapping size doesn't match number of horizontal rays");
    }

    IGN_PROFILE_BEGIN("applyMapping");
    if (!isSampleSensor)
    {
      for (unsigned int azimuthIndex = 0; azimuthIndex < this->dataPtr->horizontalRangeCount; ++azimuthIndex)
      {
        GZ_ASSERT(this->dataPtr->verticalRangeCount == this->dataPtr->mapping[azimuthIndex].size(), "cube face mapping size doesn't match number of vertical rays");
        for (unsigned int elevationIndex = 0; elevationIndex < this->dataPtr->verticalRangeCount; ++elevationIndex)
        {
          const auto& [cubeFaceId, faceCoordinates] = this->dataPtr->mapping[azimuthIndex][elevationIndex];

          GZ_ASSERT(this->dataPtr->cubeMapFaces.find(cubeFaceId) != this->dataPtr->cubeMapFaces.end(), "Required cube face has not been initialized");

          const auto &cubeFace = this->dataPtr->cubeMapFaces.at(cubeFaceId);

          // pixel coordinates
          const auto imageX = static_cast<unsigned int>(faceCoordinates.X() * (imageWidth - 1));
          const auto imageY = static_cast<unsigned int>(faceCoordinates.Y() * (imageHeight - 1));

          GZ_ASSERT(imageX < imageWidth, "Calculated image X exceeds image width");
          GZ_ASSERT(imageY < imageHeight, "Calculated image Y exceeds image height");

          const unsigned int depthImageBufferIndex = (imageY * imageWidth + imageX) * numberOfChannels;

          GZ_ASSERT(depthImageBufferIndex + 2 < cubeFace.depthImgFloat32.size(), "Depth image buffer index is out of range");

          const unsigned int laserBufferIndex = (elevationIndex * this->dataPtr->horizontalRangeCount + azimuthIndex) * numberOfChannels;
          GZ_ASSERT(laserBufferIndex + 2 < this->dataPtr->laserBuffer.size(), "Laser buffer index is out of range");
          this->dataPtr->laserBuffer[laserBufferIndex] = cubeFace.depthImgFloat32[depthImageBufferIndex];
          this->dataPtr->laserBuffer[laserBufferIndex + 1] = cubeFace.depthImgFloat32[depthImageBufferIndex + 1];
          this->dataPtr->laserBuffer[laserBufferIndex + 2] = cubeFace.depthImgFloat32[depthImageBufferIndex + 2];
        }
      }
    }
    else
    {
      for (unsigned int sampleIndex = 0; sampleIndex < this->sampleSize; sampleIndex++)
      {

        unsigned int idx = sampleIndex + sampleOffset;
        if (idx >= this->dataPtr->mapping.size())
        {
          idx = idx - this->dataPtr->mapping.size();
        }

        const auto& [cubeFaceId, faceCoordinates] = this->dataPtr->mapping[idx][0];
        GZ_ASSERT(this->dataPtr->cubeMapFaces.find(cubeFaceId) != this->dataPtr->cubeMapFaces.end(), "Required cube face has not been initialized");
        const auto &cubeFace = this->dataPtr->cubeMapFaces.at(cubeFaceId);
        const auto imageX = static_cast<unsigned int>(faceCoordinates.X() * (imageWidth - 1));
        const auto imageY = static_cast<unsigned int>(faceCoordinates.Y() * (imageHeight - 1));
        const unsigned int depthImageBufferIndex = (imageY * imageWidth + imageX) * numberOfChannels;
        const unsigned int laserBufferIndex = sampleIndex * numberOfChannels * 3;

        this->dataPtr->laserBuffer[laserBufferIndex] = Ogre::Bitwise::halfToFloat(cubeFace.depthImgFloat16[depthImageBufferIndex]); // Ogre::Bitwise::halfToFloat(
        this->dataPtr->laserBuffer[laserBufferIndex + 1] = this->dataPtr->sampleAzimuth[idx];
        this->dataPtr->laserBuffer[laserBufferIndex + 2] = this->dataPtr->sampleElevation[idx];
      }

      sampleOffset += this->sampleSize;
      if (sampleOffset > this->dataPtr->mapping.size())
      {
        sampleOffset -= this->dataPtr->mapping.size();
      }
    }
    IGN_PROFILE_END();

    IGN_PROFILE_BEGIN("copyBufferToScan");
    if (this->dataPtr->laserScan.size() != laserBufferSize) {
      this->dataPtr->laserScan.resize(laserBufferSize);
    }

    std::copy(this->dataPtr->laserBuffer.begin(),
              this->dataPtr->laserBuffer.end(),
              this->dataPtr->laserScan.begin());
    IGN_PROFILE_END();

    if (isSampleSensor)
    {
      this->dataPtr->newLaserFrame(
        this->dataPtr->laserScan.data(), this->sampleSize,
        1, 1, "BLABLA");
    }
    else
    {
      this->dataPtr->newLaserFrame(
          this->dataPtr->laserScan.data(), this->dataPtr->horizontalRangeCount,
          this->dataPtr->verticalRangeCount, 1, "BLABLA");
    }

  }

  this->newData = false;
}

/////////////////////////////////////////////////
void GpuLaser::UpdateRenderTarget(GpuLaserCubeFace &_cube_face)
{
  Ogre::RenderTarget *renderTarget = _cube_face.texture->getBuffer()->getRenderTarget();
  Ogre::SceneManager *sceneMgr = this->scene->OgreSceneManager();
  Ogre::RenderSystem *renderSys = sceneMgr->getDestinationRenderSystem();

  this->dataPtr->currentTarget = renderTarget;

  // Get pointer to the material pass
  Ogre::Pass *pass = this->dataPtr->material->getBestTechnique()->getPass(0);

  // Render the depth texture
  // OgreSceneManager::_render function automatically sets farClip to 0.
  // Which normally equates to infinite distance. We don't want this. So
  // we have to set the distance every time.
  this->camera->setFarClipDistance(static_cast<float>(this->FarClip()));

  Ogre::AutoParamDataSource autoParamDataSource;

  Ogre::Viewport *vp = renderTarget->getViewport(0);

  // Need this line to render the ground plane. No idea why it's necessary.
  renderSys->_setViewport(vp);
  sceneMgr->_setPass(pass, true, false);
  autoParamDataSource.setCurrentPass(pass);
  autoParamDataSource.setCurrentViewport(vp);
  autoParamDataSource.setCurrentRenderTarget(renderTarget);
  autoParamDataSource.setCurrentSceneManager(sceneMgr);
  autoParamDataSource.setCurrentCamera(this->camera, true);

  renderSys->setLightingEnabled(false);
  renderSys->_setFog(Ogre::FOG_NONE);

#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR == 6
  pass->_updateAutoParamsNoLights(&autoParamDataSource);
#else
  pass->_updateAutoParams(&autoParamDataSource, 1);
#endif

  // NOTE: We MUST bind parameters AFTER updating the autos
  if (pass->hasVertexProgram())
  {
    renderSys->bindGpuProgram(
        pass->getVertexProgram()->_getBindingDelegate());

#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR == 6
    renderSys->bindGpuProgramParameters(Ogre::GPT_VERTEX_PROGRAM,
    pass->getVertexProgramParameters());
#else
    renderSys->bindGpuProgramParameters(Ogre::GPT_VERTEX_PROGRAM,
      pass->getVertexProgramParameters(), 1);
#endif
  }

  if (pass->hasFragmentProgram())
  {
    renderSys->bindGpuProgram(
    pass->getFragmentProgram()->_getBindingDelegate());

#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR == 6
    renderSys->bindGpuProgramParameters(Ogre::GPT_FRAGMENT_PROGRAM,
    pass->getFragmentProgramParameters());
#else
      renderSys->bindGpuProgramParameters(Ogre::GPT_FRAGMENT_PROGRAM,
      pass->getFragmentProgramParameters(), 1);
#endif
  }

  renderTarget->update(false);
}

/////////////////////////////////////////////////
void GpuLaser::notifyRenderSingleObject(Ogre::Renderable *_rend,
      const Ogre::Pass* /*pass*/, const Ogre::AutoParamDataSource* /*source*/,
      const Ogre::LightList* /*lights*/, bool /*supp*/)
{
  Ogre::Vector4 retro = Ogre::Vector4(0, 0, 0, 0);
  try
  {
    retro = _rend->getCustomParameter(1);
  }
  catch(Ogre::ItemIdentityException& e)
  {
    _rend->setCustomParameter(1, Ogre::Vector4(0, 0, 0, 0));
  }

  Ogre::Pass *pass = this->dataPtr->material->getBestTechnique()->getPass(0);
  Ogre::RenderSystem *renderSys =
                  this->scene->OgreSceneManager()->getDestinationRenderSystem();

  Ogre::AutoParamDataSource autoParamDataSource;

  Ogre::Viewport *vp = this->dataPtr->currentTarget->getViewport(0);

  renderSys->_setViewport(vp);
  autoParamDataSource.setCurrentRenderable(_rend);
  autoParamDataSource.setCurrentPass(pass);
  autoParamDataSource.setCurrentViewport(vp);
  autoParamDataSource.setCurrentRenderTarget(this->dataPtr->currentTarget);
  autoParamDataSource.setCurrentSceneManager(this->scene->OgreSceneManager());
  autoParamDataSource.setCurrentCamera(this->camera, true);

  pass->_updateAutoParams(&autoParamDataSource,
      Ogre::GPV_GLOBAL | Ogre::GPV_PER_OBJECT);
  pass->getFragmentProgramParameters()->setNamedConstant("retro", retro[0]);
  renderSys->bindGpuProgram(
      pass->getVertexProgram()->_getBindingDelegate());

  renderSys->bindGpuProgramParameters(Ogre::GPT_VERTEX_PROGRAM,
      pass->getVertexProgramParameters(),
      Ogre::GPV_GLOBAL | Ogre::GPV_PER_OBJECT);

  renderSys->bindGpuProgram(
      pass->getFragmentProgram()->_getBindingDelegate());

  renderSys->bindGpuProgramParameters(Ogre::GPT_FRAGMENT_PROGRAM,
      pass->getFragmentProgramParameters(),
      Ogre::GPV_GLOBAL | Ogre::GPV_PER_OBJECT);
}

//////////////////////////////////////////////////
void GpuLaser::RenderImpl()
{
  IGN_PROFILE("rendering::GpuLaser::RenderImpl");
  Ogre::SceneManager *sceneMgr = this->scene->OgreSceneManager();

  sceneMgr->_suppressRenderStateChanges(true);
  sceneMgr->addRenderObjectListener(this);

  for (auto& [cubeFaceId, cubeFace] : this->dataPtr->cubeMapFaces)
  {
    this->ApplyCameraSetting(cubeFace.cameraSetting);
    this->UpdateRenderTarget(cubeFace);
    this->RevertCameraSetting(cubeFace.cameraSetting);
  }

  sceneMgr->removeRenderObjectListener(this);
  sceneMgr->_suppressRenderStateChanges(false);
}

//////////////////////////////////////////////////
GpuLaser::DataIter GpuLaser::LaserDataBegin() const
{
  const unsigned int index = 0;
  // Data stuffed into three floats (RGB)
  unsigned int skip = 3;
  // range data in R channel
  const unsigned int rangeOffset = 0;
  // intensity data in G channel
  const unsigned int intenOffset = 1;
  return {index, this->dataPtr->laserBuffer.data(), skip, rangeOffset,
          intenOffset, this->dataPtr->horizontalRangeCount};
}

//////////////////////////////////////////////////
GpuLaser::DataIter GpuLaser::LaserDataEnd() const
{

  unsigned int index;
  index = this->dataPtr->verticalRangeCount * this->dataPtr->horizontalRangeCount;

  // Data stuffed into three floats (RGB)
  unsigned int skip = 3;
  // range data in R channel
  const unsigned int rangeOffset = 0;
  // intensity data in G channel
  const unsigned int intenOffset = 1;
  return {index, this->dataPtr->laserBuffer.data(), skip, rangeOffset,
          intenOffset, this->dataPtr->horizontalRangeCount};
}

//////////////////////////////////////////////////
std::vector<float>* GpuLaser::LaserData() const
{
  return &this->dataPtr->laserBuffer;
}

//////////////////////////////////////////////////
void GpuLaser::SetUpRenderTarget(GpuLaserCubeFace &_cube_face)
{
  Ogre::RenderTarget *renderTarget = _cube_face.texture->getBuffer()->getRenderTarget();

  if (renderTarget)
  {
    // Setup the viewport to use the texture
    Ogre::Viewport *viewport = renderTarget->addViewport(this->camera);
    viewport->setClearEveryFrame(true);
    viewport->setOverlaysEnabled(false);
    viewport->setShadowsEnabled(false);
    viewport->setSkiesEnabled(false);
    viewport->setBackgroundColour(Ogre::ColourValue(static_cast<float>(this->farClip), 0.0, 1.0));
    viewport->setVisibilityMask(
        GZ_VISIBILITY_ALL & ~(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE));
  }

  this->camera->setAspectRatio(static_cast<float>(this->RayCountRatio()));
  this->camera->setFOVy(Ogre::Radian(static_cast<float>(this->CosVertFOV())));

  renderTarget->setAutoUpdated(false);
}

/////////////////////////////////////////////////
void GpuLaser::SetRangeCount(const unsigned int _w, const unsigned int _h)
{
  this->dataPtr->horizontalRangeCount = _w;
  this->dataPtr->verticalRangeCount = _h;
}

//////////////////////////////////////////////////
void GpuLaser::SetHorzHalfAngle(const double _angle)
{
  this->horzHalfAngle = _angle;
}

//////////////////////////////////////////////////
void GpuLaser::SetVertHalfAngle(const double _angle)
{
  this->vertHalfAngle = _angle;
}

//////////////////////////////////////////////////
double GpuLaser::HorzHalfAngle() const
{
  return this->horzHalfAngle;
}

//////////////////////////////////////////////////
double GpuLaser::VertHalfAngle() const
{
  return this->vertHalfAngle;
}

//////////////////////////////////////////////////
void GpuLaser::SetIsHorizontal(const bool _horizontal)
{
  this->isHorizontal = _horizontal;
}

//////////////////////////////////////////////////
bool GpuLaser::IsHorizontal() const
{
  return this->isHorizontal;
}

//////////////////////////////////////////////////
double GpuLaser::HorzFOV() const
{
  return this->hfov;
}

//////////////////////////////////////////////////
double GpuLaser::VertFOV() const
{
  return this->vfov;
}

//////////////////////////////////////////////////
void GpuLaser::SetHorzFOV(const double _hfov)
{
  this->hfov = _hfov;
}

//////////////////////////////////////////////////
void GpuLaser::SetVertFOV(const double _vfov)
{
  this->vfov = _vfov;
}

//////////////////////////////////////////////////
double GpuLaser::CosHorzFOV() const
{
  return this->chfov;
}

//////////////////////////////////////////////////
void GpuLaser::SetCosHorzFOV(const double _chfov)
{
  this->chfov = _chfov;
}

//////////////////////////////////////////////////
double GpuLaser::CosVertFOV() const
{
  return this->cvfov;
}

//////////////////////////////////////////////////
void GpuLaser::SetCosVertFOV(const double _cvfov)
{
  this->cvfov = _cvfov;
}

//////////////////////////////////////////////////
double GpuLaser::NearClip() const
{
  return this->nearClip;
}

//////////////////////////////////////////////////
double GpuLaser::FarClip() const
{
  return this->farClip;
}

//////////////////////////////////////////////////
void GpuLaser::SetNearClip(const double _near)
{
  this->nearClip = _near;
}

//////////////////////////////////////////////////
void GpuLaser::SetFarClip(const double _far)
{
  this->farClip = _far;
}

//////////////////////////////////////////////////
unsigned int GpuLaser::CameraCount() const
{
  return this->cameraCount;
}

//////////////////////////////////////////////////
void GpuLaser::SetCameraCount(const unsigned int _cameraCount)
{
  /*unused*/
}

//////////////////////////////////////////////////
double GpuLaser::RayCountRatio() const
{
  return this->rayCountRatio;
}

//////////////////////////////////////////////////
void GpuLaser::SetRayCountRatio(const double _rayCountRatio)
{
  this->rayCountRatio = _rayCountRatio;
}

//////////////////////////////////////////////////
void GpuLaser::SetIsSample(const bool _sampleSensor)
{
  this->isSampleSensor = _sampleSensor;
}

//////////////////////////////////////////////////
void GpuLaser::SetSampleSize(const unsigned int _sampleSize)
{
  this->sampleSize = _sampleSize;
}

//////////////////////////////////////////////////
event::ConnectionPtr GpuLaser::ConnectNewLaserFrame(
    std::function<void (const float *_frame, unsigned int _width,
    unsigned int _height, unsigned int _depth,
    const std::string &_format)> _subscriber)
{
  return this->dataPtr->newLaserFrame.Connect(_subscriber);
}

//////////////////////////////////////////////////
void GpuLaser::ApplyCameraSetting(const GpuLaserCameraOrientationOffset &_setting)
{
  this->sceneNode->roll(Ogre::Radian(static_cast<float>(_setting.azimuthOffset)));
  this->sceneNode->yaw(Ogre::Radian(static_cast<float>(_setting.elevationOffset)));
}

//////////////////////////////////////////////////
void GpuLaser::RevertCameraSetting(const GpuLaserCameraOrientationOffset &_setting)
{
  this->sceneNode->yaw(Ogre::Radian(static_cast<float>(-_setting.elevationOffset)));
  this->sceneNode->roll(Ogre::Radian(static_cast<float>(-_setting.azimuthOffset)));
}

//////////////////////////////////////////////////
void GpuLaser::InitMapping(const std::vector<double> &_azimuth_values, const std::vector<double> &_elevation_values)
{
  this->dataPtr->mapping.clear();
  this->dataPtr->mapping.reserve(_azimuth_values.size());

  GZ_ASSERT(_azimuth_values.size() == _elevation_values.size(), "number of azimuth values doesn't match number of elevation value");
  if (_azimuth_values.empty())
  {
    return;
  }

  double min_azimuth = -M_PI;
  if (!isSampleSensor)
  {
    min_azimuth = *std::min_element(_azimuth_values.begin(), _azimuth_values.end());
  }

  const double front_face_azimuth = min_azimuth + M_PI_4;

  // define face orientations w.r.t. front face
  const std::map<GpuLaserCubeFaceId, GpuLaserCameraOrientationOffset> cube_camera_orientations {
      {GpuLaserCubeFaceId::CUBE_FRONT_FACE, {front_face_azimuth, 0.}},
      {GpuLaserCubeFaceId::CUBE_LEFT_FACE, {front_face_azimuth + M_PI_2, 0.0}},
      {GpuLaserCubeFaceId::CUBE_REAR_FACE, {front_face_azimuth + M_PI, 0.0}},
      {GpuLaserCubeFaceId::CUBE_RIGHT_FACE, {front_face_azimuth + M_PI + M_PI_2, 0.0}},
      {GpuLaserCubeFaceId::CUBE_TOP_FACE, {front_face_azimuth, -M_PI_2}},
      {GpuLaserCubeFaceId::CUBE_BOTTOM_FACE, {front_face_azimuth, M_PI_2}},
  };
  for (size_t i = 0; i < _azimuth_values.size(); ++i)
  {
    const double azimuth = _azimuth_values.at(i);
    this->dataPtr->mapping.emplace_back();
    const double elevation = _elevation_values.at(i);
    const auto& [cube_face_id, face_coordinates] = GpuLaser::FindCubeFaceMapping(azimuth - min_azimuth, elevation);
    this->dataPtr->mapping.back().emplace_back(cube_face_id, face_coordinates);

    // create cube map faces if necessary
    if (this->dataPtr->cubeMapFaces.find(cube_face_id) ==
        this->dataPtr->cubeMapFaces.end()) {
      GpuLaserCubeFace face;
      face.cameraSetting = cube_camera_orientations.at(cube_face_id);
      this->dataPtr->cubeMapFaces.insert({cube_face_id, face});
    }
  }
  this->cameraCount = this->dataPtr->cubeMapFaces.size();
  this->dataPtr->sampleAzimuth = _azimuth_values;
  this->dataPtr->sampleElevation = _elevation_values;
}

//////////////////////////////////////////////////
void GpuLaser::InitMapping(const std::set<double> &_azimuth_values, const std::set<double> &_elevation_values)
{
  this->dataPtr->mapping.clear();
  this->dataPtr->mapping.reserve(_azimuth_values.size());

  if (_azimuth_values.empty())
  {
    return;
  }

  const double min_azimuth = *_azimuth_values.begin();
  const double front_face_azimuth = min_azimuth + M_PI_4;

  // define face orientations w.r.t. front face
  const std::map<GpuLaserCubeFaceId, GpuLaserCameraOrientationOffset> cube_camera_orientations {
      {GpuLaserCubeFaceId::CUBE_FRONT_FACE, {front_face_azimuth, 0.}},
      {GpuLaserCubeFaceId::CUBE_LEFT_FACE, {front_face_azimuth + M_PI_2, 0.0}},
      {GpuLaserCubeFaceId::CUBE_REAR_FACE, {front_face_azimuth + M_PI, 0.0}},
      {GpuLaserCubeFaceId::CUBE_RIGHT_FACE, {front_face_azimuth + M_PI + M_PI_2, 0.0}},
      {GpuLaserCubeFaceId::CUBE_TOP_FACE, {front_face_azimuth, -M_PI_2}},
      {GpuLaserCubeFaceId::CUBE_BOTTOM_FACE, {front_face_azimuth, M_PI_2}},
  };
  for (const double azimuth : _azimuth_values)
  {
    this->dataPtr->mapping.emplace_back();
    this->dataPtr->mapping.back().reserve(_elevation_values.size());

    for (const double elevation : _elevation_values)
    {
      const auto& [cube_face_id, face_coordinates] = GpuLaser::FindCubeFaceMapping(azimuth - min_azimuth, elevation);
      this->dataPtr->mapping.back().emplace_back(cube_face_id, face_coordinates);

      // create cube map faces if necessary
      if (this->dataPtr->cubeMapFaces.find(cube_face_id) ==
          this->dataPtr->cubeMapFaces.end()) {
        GpuLaserCubeFace face;
        face.cameraSetting = cube_camera_orientations.at(cube_face_id);
        this->dataPtr->cubeMapFaces.insert({cube_face_id, face});
      }
    }
  }

  this->cameraCount = this->dataPtr->cubeMapFaces.size();
}

//////////////////////////////////////////////////
GpuLaserCubeMappingPoint GpuLaser::FindCubeFaceMapping(const double _azimuth, const double _elevation)
{
  if (_azimuth < 0)
  {
    throw std::runtime_error("Azimuth angle should be relative to minimum angle, i.e. it must not be negative!");
  }

  const GpuLaserCubeFaceId face_id = GpuLaser::FindCubeFace(_azimuth, _elevation);

  // center point of the face plane
  ignition::math::Vector3d plane_point;
  switch (face_id)
  {
    case GpuLaserCubeFaceId::CUBE_FRONT_FACE:
      plane_point = {0.5, 0., 0.};
      break;
    case GpuLaserCubeFaceId::CUBE_LEFT_FACE:
      plane_point = {0., 0.5, 0.};
      break;
    case GpuLaserCubeFaceId::CUBE_REAR_FACE:
      plane_point = {-0.5, 0., 0.};
      break;
    case GpuLaserCubeFaceId::CUBE_RIGHT_FACE:
      plane_point = {0., -0.5, 0.};
      break;
    case GpuLaserCubeFaceId::CUBE_TOP_FACE:
      plane_point = {0., 0., 0.5};
      break;
    case GpuLaserCubeFaceId::CUBE_BOTTOM_FACE:
      plane_point = {0., 0., -0.5};
      break;
    default:
      throw std::runtime_error("Invalid face ID");
  }

  const ignition::math::Vector3d plane_normal = plane_point.Normalized();
  const ignition::math::Vector3d viewing_ray = GpuLaser::ViewingRay(_azimuth, _elevation);

  // calculate intersection of viewing ray with cube face plane
  const double s = (-plane_normal).Dot(-plane_point) / plane_normal.Dot(viewing_ray);

  // offset from face center to intersection
  const ignition::math::Vector3d intersection_offset = s * viewing_ray - plane_point;

  // offset in the 2D image on the face
  ignition::math::Vector2d intersection_image_offset;
  switch (face_id)
  {
    case GpuLaserCubeFaceId::CUBE_FRONT_FACE:
      intersection_image_offset = {-intersection_offset.Y(),
                                   -intersection_offset.Z()};
      break;
    case GpuLaserCubeFaceId::CUBE_LEFT_FACE:
      intersection_image_offset = {intersection_offset.X(),
                                   -intersection_offset.Z()};
      break;
    case GpuLaserCubeFaceId::CUBE_REAR_FACE:
      intersection_image_offset = {intersection_offset.Y(),
                                   -intersection_offset.Z()};
      break;
    case GpuLaserCubeFaceId::CUBE_RIGHT_FACE:
      intersection_image_offset = {-intersection_offset.X(),
                                   -intersection_offset.Z()};
      break;
    case GpuLaserCubeFaceId::CUBE_TOP_FACE:
      intersection_image_offset = {-intersection_offset.Y(),
                                   intersection_offset.X()};
      break;
    case GpuLaserCubeFaceId::CUBE_BOTTOM_FACE:
      intersection_image_offset = {-intersection_offset.Y(),
                                   -intersection_offset.X()};
      break;
    default:
      throw std::runtime_error("Invalid face ID");
  }

  // shift offset from image center to origin
  intersection_image_offset += {0.5, 0.5};

  intersection_image_offset.Set(ignition::math::clamp(intersection_image_offset.X(), 0., 1.),
                                ignition::math::clamp(intersection_image_offset.Y(), 0., 1.));

  return {face_id, intersection_image_offset};
}

//////////////////////////////////////////////////
GpuLaserCubeFaceId GpuLaser::FindCubeFace(const double _azimuth, const double _elevation)
{
  const ignition::math::Vector3d v = GpuLaser::ViewingRay(_azimuth, _elevation);

  // find corresponding cube face
  if (std::abs(v.Z()) > std::abs(v.X()) && std::abs(v.Z()) > std::abs(v.Y()))
  {
    if (v.Z() >= 0)
    {
      return GpuLaserCubeFaceId::CUBE_TOP_FACE;
    }
    else
    {
      return GpuLaserCubeFaceId::CUBE_BOTTOM_FACE;
    }
  }
  else if (_azimuth < M_PI_2)
  {
    return GpuLaserCubeFaceId::CUBE_FRONT_FACE;
  }
  else if (_azimuth >= M_PI_2 && _azimuth < M_PI)
  {
    return GpuLaserCubeFaceId::CUBE_LEFT_FACE;
  }
  else if (_azimuth >= M_PI && _azimuth < 3. * M_PI_2)
  {
    return GpuLaserCubeFaceId::CUBE_REAR_FACE;
  }
  else
  {
    return GpuLaserCubeFaceId::CUBE_RIGHT_FACE;
  }
}

//////////////////////////////////////////////////
ignition::math::Vector3d GpuLaser::ViewingRay(const double _azimuth, const double _elevation)
{
  /// subtracting M_PI_4 from azimuth because cube face horizontal center correspondents to azimuth = M_PI_4
  return {std::cos(_azimuth - M_PI_4) * std::cos(_elevation),
          std::sin(_azimuth - M_PI_4) * std::cos(_elevation),
          std::sin(_elevation)};
}
