/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "gazebo/common/Assert.hh"
#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/Camera.hh"
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
  this->dataPtr->distortionScale = math::Vector2d(1.0, 1.0);
  this->dataPtr->distortionCrop = true;
}

//////////////////////////////////////////////////
Distortion::~Distortion()
{
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

  if (this->dataPtr->k1 >= 0)
  {
    gzerr << "Pincushion model is currently not supported."
      << " Please use a negative k1 coefficient for barrel distortion"
      << std::endl;
  }
}

//////////////////////////////////////////////////
void Distortion::SetCamera(CameraPtr _camera)
{
  if (!_camera)
  {
    gzerr << "Unable to apply distortion, camera is NULL" << std::endl;
    return;
  }

  if (this->dataPtr->k1 >= 0)
  {
    gzerr << "Currently only Barrel Distortion is supported. "
        << "Distortion will not be applied." << std::endl;
    return;
  }

  // seems to work best with a square distortion map texture
  unsigned int texSide = _camera->GetImageHeight() > _camera->GetImageWidth() ?
      _camera->GetImageHeight() : _camera->GetImageWidth();
  unsigned int texWidth = texSide;
  unsigned int texHeight = texSide;
  unsigned int imageSize = texWidth * texHeight;

  this->dataPtr->distortionMap.resize(imageSize);
  for (unsigned int i = 0; i < this->dataPtr->distortionMap.size(); ++i)
    this->dataPtr->distortionMap[i] = -1;

  double incrU = 1.0 / texWidth;
  double incrV = 1.0 / texHeight;

  // obtain bounds of the distorted image points.
  math::Vector2d boundA = this->Distort(math::Vector2d(0, 0),
      this->dataPtr->lensCenter,
      this->dataPtr->k1, this->dataPtr->k2, this->dataPtr->k3,
      this->dataPtr->p1, this->dataPtr->p2);
  math::Vector2d boundB = this->Distort(math::Vector2d(1, 1),
      this->dataPtr->lensCenter,
      this->dataPtr->k1, this->dataPtr->k2, this->dataPtr->k3,
      this->dataPtr->p1, this->dataPtr->p2);

  if (this->dataPtr->distortionCrop)
    this->dataPtr->distortionScale = boundB - boundA;

  // fill the distortion map
  for (unsigned int i = 0; i < texHeight; ++i)
  {
    double v = i*incrU;
    for (unsigned int j = 0; j < texWidth; ++j)
    {
      double u = j*incrV;
      math::Vector2d uv(u, v);
      math::Vector2d out = this->Distort(uv, this->dataPtr->lensCenter,
          this->dataPtr->k1, this->dataPtr->k2, this->dataPtr->k3,
          this->dataPtr->p1, this->dataPtr->p2);

      // compute the index in the distortion map
      unsigned int idxU = out.x * texWidth;
      unsigned int idxV = out.y * texHeight;
      unsigned int mapIdx = idxV * texWidth + idxU;

      // this should not happen for barrel distortion as the normalized
      // distorted coordinate should be within (0, 0) and (1.0, 1.0).
      if (mapIdx >= imageSize)
      {
        gzlog << "Warning: Normalized distorted coordinate is out of range."
            << " Index: '" << mapIdx << "' vs Image Size: '" << imageSize << "'"
            << " This should not happen for barrel distortion" << std::endl;
        continue;
      }

      this->dataPtr->distortionMap[mapIdx] = uv;
    }
  }

  // Apply interpolation to the resulting distortion map.
  // This is mostly needed for barrel distortion where the the center of the
  // distortion texture may contain a few black pixels.
  unsigned int boundAIdxU = boundA.x * texWidth;
  unsigned int boundAIdxV = boundA.y * texHeight;
  unsigned int boundBIdxU = boundB.x * texWidth;
  unsigned int boundBIdxV = boundB.y * texHeight;
  // limit interpolation to the boundary formed by the distorted image points.
  unsigned int roiWidth = boundBIdxU - boundAIdxU;
  unsigned int roiHeight = boundBIdxV - boundAIdxV;
  for (unsigned int i  = 0 ; i < roiHeight; ++i)
  {
    for (unsigned int j  = 0 ; j < roiWidth; ++j)
    {
      unsigned int mapIdx = (boundAIdxV + i)  * texWidth + boundAIdxU + j;
      // check for empty mapping within the region and correct it by
      // interpolating four neighboring distortion map values.
      if (this->dataPtr->distortionMap[mapIdx] == math::Vector2d(-1, -1))
      {
        math::Vector2d interpolate(0, 0);
        int sampleSize = 0;
        // left
        if ((boundAIdxU + j) != 0 &&
            !(this->dataPtr->distortionMap[mapIdx-1] == math::Vector2d(-1, -1)))
        {
          interpolate += this->dataPtr->distortionMap[mapIdx-1];
          sampleSize++;
        }
        // right
        if ((boundAIdxU + j+1) < texWidth &&
            !(this->dataPtr->distortionMap[mapIdx+1] == math::Vector2d(-1, -1)))
        {
          interpolate += this->dataPtr->distortionMap[mapIdx+1];
          sampleSize++;
        }
        // top
        if ((boundAIdxV + i) != 0)
        {
          unsigned int topIdx =
              (boundAIdxV + i-1) * texWidth + boundAIdxU + j;
          if (!(this->dataPtr->distortionMap[topIdx]
              == math::Vector2d(-1, -1)))
          {
            interpolate += this->dataPtr->distortionMap[mapIdx-1];
            sampleSize++;
          }
        }
        // bottom
        if ((boundAIdxV + i+1) < texHeight)
        {
          unsigned int bottomIdx =
              (boundAIdxV + i+1) * texWidth + boundAIdxU + j;
          if (!(this->dataPtr->distortionMap[bottomIdx]
              == math::Vector2d(-1, -1)))
          {
            interpolate += this->dataPtr->distortionMap[mapIdx+1];
            sampleSize++;
          }
        }
        interpolate.x = interpolate.x / sampleSize;
        interpolate.y = interpolate.y / sampleSize;
        this->dataPtr->distortionMap[mapIdx] = interpolate;
      }
    }
  }

  // set up the compositor
  Ogre::MaterialPtr distMat =
      Ogre::MaterialManager::getSingleton().getByName(
      "Gazebo/CameraDistortionMap");
  this->dataPtr->lensDistortionInstance =
      Ogre::CompositorManager::getSingleton().addCompositor(
      _camera->GetViewport(), "CameraDistortionMap/Default");
  this->dataPtr->lensDistortionInstance->setEnabled(true);

  // create the distortion map texture
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

  // fill the distortion map
  pixelBuffer->lock(Ogre::HardwareBuffer::HBL_NORMAL);
  const Ogre::PixelBox &pixelBox = pixelBuffer->getCurrentLock();
  float *pDest = static_cast<float *>(pixelBox.data);
  for (unsigned int i = 0; i < texHeight; ++i)
  {
    for (unsigned int j = 0; j < texWidth; ++j)
    {
      math::Vector2d vec =
          this->dataPtr->distortionMap[i*texWidth+j];
      *pDest++ = vec.x;
      *pDest++ = vec.y;
      *pDest++ = 0;
    }
  }
  pixelBuffer->unlock();

  // pass a scale param to the pixel shader for scaling the texture in order to
  // remove black border.
  Ogre::GpuProgramParametersSharedPtr params =
      distMat->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
  params->setNamedConstant("scale",
      Ogre::Vector3(this->dataPtr->distortionScale.x,
      this->dataPtr->distortionScale.y, 1.0));

  // set up the distortion map texture to be used in the pixel shader.
  distMat->getTechnique(0)->getPass(0)->createTextureUnitState(texName, 1);
}

//////////////////////////////////////////////////
math::Vector2d Distortion::Distort(const math::Vector2d &_in,
    const math::Vector2d &_center, double _k1, double _k2, double _k3,
    double _p1, double _p2)
{
  // apply Brown's distortion model, see
  // http://en.wikipedia.org/wiki/Distortion_%28optics%29#Software_correction

  math::Vector2d normalized2d = _in - _center;
  math::Vector3 normalized(normalized2d.x, normalized2d.y, 0);
  double rSq = normalized.x * normalized.x
      + normalized.y * normalized.y;

  // radial
  math::Vector3 dist = normalized * (1.0 +
      _k1 * rSq +
      _k2 * rSq * rSq +
      _k3 * rSq * rSq * rSq);

  // tangential
  dist.x += _p2 * (rSq + 2 * (normalized.x*normalized.x)) +
      2 * _p1 * normalized.x * normalized.y;
  dist.y += _p1 * (rSq + 2 * (normalized.y*normalized.y)) +
      2 * _p2 * normalized.x * normalized.y;
  math::Vector2d out = _center + math::Vector2d(dist.x, dist.y);

  return out;
}

//////////////////////////////////////////////////
void Distortion::SetCrop(bool _crop)
{
  this->dataPtr->distortionCrop = _crop;
}

//////////////////////////////////////////////////
double Distortion::GetK1() const
{
  return this->dataPtr->k1;
}

//////////////////////////////////////////////////
double Distortion::GetK2() const
{
  return this->dataPtr->k2;
}

//////////////////////////////////////////////////
double Distortion::GetK3() const
{
  return this->dataPtr->k3;
}

//////////////////////////////////////////////////
double Distortion::GetP1() const
{
  return this->dataPtr->p1;
}

//////////////////////////////////////////////////
double Distortion::GetP2() const
{
  return this->dataPtr->p2;
}

//////////////////////////////////////////////////
math::Vector2d Distortion::GetCenter() const
{
  return this->dataPtr->lensCenter;
}
