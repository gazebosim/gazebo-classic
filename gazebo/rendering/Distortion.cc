/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#include <sdf/sdf.hh>

#include "gazebo/common/Events.hh"
#include "gazebo/common/Assert.hh"
#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/DistortionPrivate.hh"
#include "gazebo/rendering/Distortion.hh"

using namespace gazebo;
using namespace rendering;

//////////////////////////////////////////////////
Distortion::Distortion()
  : dataPtr(new DistortionPrivate)
{
  this->dataPtr->k1 = 0;
  this->dataPtr->k2 = 0;
  this->dataPtr->k3 = 0;
  this->dataPtr->p1 = 0;
  this->dataPtr->p2 = 0;
  this->dataPtr->lensCenter = math::Vector2d(0.5, 0.5);
  this->dataPtr->distortionCrop = false;
}

//////////////////////////////////////////////////
Distortion::~Distortion()
{
  this->sdf.reset();

  delete this->dataPtr;
  this->dataPtr = NULL;
}

//////////////////////////////////////////////////
void Distortion::Load(sdf::ElementPtr _sdf)
{
  this->sdf = _sdf;
  this->dataPtr->k1 = this->sdf->Get<double>("k1");
  this->dataPtr->k2 = this->sdf->Get<double>("k2");
  this->dataPtr->k3 = this->sdf->Get<double>("k3");
  this->dataPtr->p1 = this->sdf->Get<double>("p1");
  this->dataPtr->p2 = this->sdf->Get<double>("p2");
  this->dataPtr->lensCenter = this->sdf->Get<math::Vector2d>("center");

  if (this->dataPtr->k1 > 0)
  {
    gzerr << "Pincushion model is currently not supported."
      << " Please use a negative K1 coefficient" << std::endl;
  }

  this->dataPtr->distortionCrop = false;
}

//////////////////////////////////////////////////
void Distortion::SetCamera(CameraPtr _camera)
{
  GZ_ASSERT(_camera, "Unable to apply distortion, camera is NULL");

  this->dataPtr->distortionScale.x = 1.0;
  this->dataPtr->distortionScale.y = 1.0;

  // seems to work best with a square distortion map texture
  unsigned int texSide = _camera->GetImageHeight() > _camera->GetImageWidth() ?
      _camera->GetImageHeight() : _camera->GetImageWidth();
  unsigned int texWidth = texSide;
  unsigned int texHeight = texSide;
  unsigned int imageSize = texWidth * texHeight;

  std::vector<math::Vector2d> uvMap;
  uvMap.resize(imageSize);
  this->dataPtr->distortionMap.resize(imageSize);
  for (unsigned int i = 0; i < this->dataPtr->distortionMap.size(); ++i)
    this->dataPtr->distortionMap[i] = -1;

  double incrU = 1.0 / texWidth ;
  double incrV = 1.0 / texHeight;

  for (unsigned int i = 0; i < texHeight; ++i)
  {
    double v = i*incrU;
    for (unsigned int j = 0; j < texWidth; ++j)
    {
      double u = j*incrV;
      math::Vector2d texCoord(u, v);
      math::Vector2d normalized2d = texCoord - this->dataPtr->lensCenter;
      math::Vector3 normalized(normalized2d.x, normalized2d.y, 0);
      double rSq = normalized.x * normalized.x
          + normalized.y * normalized.y;
      math::Vector3 dist = normalized * ( 1.0 +
          this->dataPtr->k1 * rSq +
          this->dataPtr->k2 * rSq * rSq +
          this->dataPtr->k3 * rSq * rSq * rSq);
      dist.x += this->dataPtr->p1
          * (rSq + 2 * (normalized.x*normalized.x)) +
          2 * this->dataPtr->p2
          * normalized.x * normalized.y;
      dist.y += this->dataPtr->p2
          * (rSq + 2 * (normalized.y*normalized.y)) +
          2 * this->dataPtr->p1
          * normalized.x * normalized.y;
      math::Vector2d out = this->dataPtr->lensCenter
          + math::Vector2d(dist.x, dist.y);

      // fill the distortion map
      int idxU = out.x * texHeight;
      int idxV = out.y * texWidth;
      int mapIdx = idxV * texWidth + idxU;
      math::Vector2d uv(u, v);

      this->dataPtr->distortionMap[mapIdx] = uv;
      uvMap[i*texWidth + j] = uv;
    }
  }

  Ogre::MaterialPtr distMat =
      Ogre::MaterialManager::getSingleton().getByName(
      "Gazebo/CameraDistortionMap");
/*  Ogre::GpuProgramParametersSharedPtr params =
      distMat->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
  params->setNamedConstant("k1", static_cast<Ogre::Real>(this->dataPtr->k1));
  params->setNamedConstant("k2", static_cast<Ogre::Real>(this->dataPtr->k2));
  params->setNamedConstant("k3", static_cast<Ogre::Real>(this->dataPtr->k3));
  params->setNamedConstant("p1", static_cast<Ogre::Real>(this->dataPtr->p1));
  params->setNamedConstant("p2", static_cast<Ogre::Real>(this->dataPtr->p2));
  params->setNamedConstant("center",
      static_cast<Ogre::Vector3>(
      Ogre::Vector3(this->dataPtr->lensCenter.x,
      this->dataPtr->lensCenter.y, 0.0)));
  params->setNamedConstant("scale",
      static_cast<Ogre::Vector3>(
      Ogre::Vector3(1.0, 1.0, 1.0)));
  params->setNamedConstant("scaleIn",
      static_cast<Ogre::Vector3>(
      Ogre::Vector3(this->dataPtr->distortionScale.x,
      this->dataPtr->distortionScale.y, 0.0)));*/
  this->dataPtr->lensDistortionInstance =
      Ogre::CompositorManager::getSingleton().addCompositor(
      _camera->GetViewport(), "CameraDistortionMap/Default");
  this->dataPtr->lensDistortionInstance->setEnabled(true);

  // Create the ditortion map texture
  std::string texName = _camera->GetName() + "_distortionTex";
  Ogre::TexturePtr renderTexture =
      Ogre::TextureManager::getSingleton().createManual(
          texName,
          "General",
          Ogre::TEX_TYPE_2D,
          texWidth,
          texHeight,
          0,
          Ogre::PF_FLOAT32_RGB);
  Ogre::HardwarePixelBufferSharedPtr pixelBuffer = renderTexture->getBuffer();

  // Lock the pixel buffer and get a pixel box
  pixelBuffer->lock(Ogre::HardwareBuffer::HBL_NORMAL);
  const Ogre::PixelBox &pixelBox = pixelBuffer->getCurrentLock();

  float *pDest = static_cast<float *>(pixelBox.data);

  for (unsigned int i = 0; i < texHeight; ++i)
  {
    for(unsigned int j = 0; j < texWidth; ++j)
    {
      math::Vector2d vec =
          this->dataPtr->distortionMap[i*texWidth+j];
      *pDest++ = vec.x;
      *pDest++ = vec.y;
      *pDest++ = 0;
    }
  }

  // Unlock the pixel buffer
  pixelBuffer->unlock();

//  Ogre::TextureUnitState *textureUnitState =
      distMat->getTechnique(0)->getPass(0)->createTextureUnitState(texName, 1);
}

//////////////////////////////////////////////////
void Distortion::Fini()
{
}
