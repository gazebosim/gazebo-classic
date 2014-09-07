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

  // for barrel distortion - crop the black pixels surrounding the
  // distorted image.
  //if (this->dataPtr->k1 > 0)
    this->dataPtr->distortionCrop = true;
}

//////////////////////////////////////////////////
void Distortion::SetCamera(CameraPtr _camera)
{
  GZ_ASSERT(_camera, "Unable to apply distortion, camera is NULL");

  this->dataPtr->distortionScale.x = 1.0;
  this->dataPtr->distortionScale.y = 1.0;

  /*gned int distortionMapSize =
      _camera->GetImageHeight()*_camera->GetImageWidth()*2;
  float *distortionMap = new float[distortionMapSize];

  for (unsigned int i = 0; i < distortionMapSize; ++i)
    distortionMap[i] = -1;*/

  std::vector<math::Vector2d> map;
  map.resize(_camera->GetImageHeight()*_camera->GetImageWidth());
  for (unsigned int i = 0; i < map.size(); ++i)
    map[i] = -1;

/*  math::Vector2d remap[_camera->GetImageWidth()][_camera->GetImageHeight()];
  for (unsigned int i = 0; i < _camera->GetImageWidth(); ++i)
    for (unsigned int j = 0; j < _camera->GetImageHeight(); ++j)
      remap[i][j] = -1;*/

  // crop the black borders for barrel distortion
  if (this->dataPtr->distortionCrop)
  {
    double trX = 0;
    double trY = 0;
    double blX = 0;
    double blY = 0;
    double dtr = GZ_DBL_MAX;
    double dbl = GZ_DBL_MAX;
    double incrU = 1.0 / (_camera->GetImageWidth());
    double incrV = 1.0 / (_camera->GetImageHeight());
    for (unsigned int i = 0; i <= _camera->GetImageWidth(); ++i)
    {
      double u = i*incrU;
      for (unsigned int j = 0; j <= _camera->GetImageHeight(); ++j)
      {
        double v = j*incrV;
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

        //std::cerr << out.x << " " << out.y << std::endl;

        // fill the distortion map
        int idxU = out.x * _camera->GetImageWidth();
        int idxV = out.y * _camera->GetImageHeight();
        std::cerr << " idx " << idxU << " " << idxV << std::endl;

        map[idxU * _camera->GetImageHeight() + idxV] = math::Vector2d(u, v);

        double dx = out.x;
        double dy = out.y;

        if (dx >= 0 && dy >= 0 && (dx + dy) < dtr)
        {
          trX = u;
          trY = v;
          dtr = dx + dy;
        }
        dx = out.x - 1.0;
        dy = out.y - 1.0;
        if (dx <= 0 && dy <= 0 && (fabs(dx) + fabs(dy)) < dbl)
        {
          blX = u;
          blY = v;
          dbl = (fabs(dx) + fabs(dy));
        }
      }
    }

    this->dataPtr->distortionScale.x = blX - trX;
    this->dataPtr->distortionScale.y = blY - trY;
  }

  Ogre::MaterialPtr distMat =
      Ogre::MaterialManager::getSingleton().getByName(
      "Gazebo/CameraDistortion");
  Ogre::GpuProgramParametersSharedPtr params =
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
      this->dataPtr->distortionScale.y, 0.0)));
  this->dataPtr->lensDistortionInstance =
      Ogre::CompositorManager::getSingleton().addCompositor(
      _camera->GetViewport(), "CameraDistortion/Default");
  this->dataPtr->lensDistortionInstance->setEnabled(true);

  // Create the render texture
  std::string texName = _camera->GetName() + "_distortionTex";
  Ogre::TexturePtr renderTexture = Ogre::TextureManager::getSingleton().createManual(
      texName,
      "General",
      Ogre::TEX_TYPE_2D,
      _camera->GetImageWidth(),
      _camera->GetImageHeight(),
      0,
      Ogre::PF_FLOAT32_RGB);
  Ogre::HardwarePixelBufferSharedPtr pixelBuffer = renderTexture->getBuffer();

  // Lock the pixel buffer and get a pixel box
  pixelBuffer->lock(Ogre::HardwareBuffer::HBL_NORMAL);
  const Ogre::PixelBox &pixelBox = pixelBuffer->getCurrentLock();

  float *pDest = static_cast<float *>(pixelBox.data);

  for (unsigned int i = 0; i < _camera->GetImageWidth(); ++i)
  {
    for(unsigned int j = 0; j < _camera->GetImageHeight(); ++j)
    {
      math::Vector2d vec =
//            this->dataPtr->distortionMap[i*_camera->GetImageHeight()+j];
          math::Vector2d(1, 0);
      *pDest++ = 1;//vec.x;
      *pDest++ = 0;//vec.y;
      *pDest++ = 0;
    }
    //pDest += pixelBox.getRowSkip() * Ogre::PixelUtil::getNumElemBytes(pixelBox.format);
  }

  // Unlock the pixel buffer
  pixelBuffer->unlock();

  Ogre::TextureUnitState *textureUnitState =
      distMat->getTechnique(0)->getPass(0)->createTextureUnitState(texName, 1);
}

//////////////////////////////////////////////////
void Distortion::Fini()
{
}
