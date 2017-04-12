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
  this->dataPtr->distortionCrop = false;
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

  if (this->dataPtr->k1 < 0)
  {
    this->dataPtr->distortionCrop = true;
  }
  else
  {
    this->dataPtr->distortionCrop = false;
  }
}

//////////////////////////////////////////////////
math::Vector2d Distortion::GetDistortionMapValueClamped(int x, int y) const
{
  if (x < 0 || x >= static_cast<int>(this->dataPtr->distortionTexWidth) ||
      y < 0 || y >= static_cast<int>(this->dataPtr->distortionTexHeight))
  {
    return math::Vector2d(-1, -1);
  }
  math::Vector2d res =
      this->dataPtr->distortionMap[y*this->dataPtr->distortionTexWidth+x];
  return res;
}

//////////////////////////////////////////////////
void Distortion::SetCamera(CameraPtr _camera)
{
  if (!_camera)
  {
    gzerr << "Unable to apply distortion, camera is NULL" << std::endl;
    return;
  }


  // seems to work best with a square distortion map texture
  unsigned int texSide = _camera->ImageHeight() > _camera->ImageWidth() ?
      _camera->ImageHeight() : _camera->ImageWidth();
  this->dataPtr->distortionTexWidth = texSide+1;
  this->dataPtr->distortionTexHeight = texSide+1;
  unsigned int imageSize =
      this->dataPtr->distortionTexWidth * this->dataPtr->distortionTexHeight;
  double incrU = 1.0 / this->dataPtr->distortionTexWidth;
  double incrV = 1.0 / this->dataPtr->distortionTexHeight;

  // initialize distortion map
  this->dataPtr->distortionMap.resize(imageSize);
  for (unsigned int i = 0; i < this->dataPtr->distortionMap.size(); ++i)
  {
    this->dataPtr->distortionMap[i] = -1;
  }

  // fill the distortion map
  for (unsigned int i = 0; i < this->dataPtr->distortionTexHeight; ++i)
  {
    double v = i*incrV;
    for (unsigned int j = 0; j < this->dataPtr->distortionTexWidth; ++j)
    {
      double u = j*incrU;
      math::Vector2d uv(u, v);
      math::Vector2d out = this->Distort(uv, this->dataPtr->lensCenter,
          this->dataPtr->k1, this->dataPtr->k2, this->dataPtr->k3,
          this->dataPtr->p1, this->dataPtr->p2);

      // compute the index in the distortion map
      unsigned int idxU = out.x * this->dataPtr->distortionTexWidth;
      unsigned int idxV = out.y * this->dataPtr->distortionTexHeight;

      if (idxU < this->dataPtr->distortionTexWidth &&
          idxV < this->dataPtr->distortionTexHeight)
      {
        unsigned int mapIdx = idxV * this->dataPtr->distortionTexWidth + idxU;
        this->dataPtr->distortionMap[mapIdx] = uv;
      }
      // else: pixel maps outside the image bounds.
      // This is expected and normal to ensure
      // no black borders; carry on
    }
  }

  // set up the distortion instance
  Ogre::MaterialPtr distMat =
      Ogre::MaterialManager::getSingleton().getByName(
      "Gazebo/CameraDistortionMap");
  this->dataPtr->lensDistortionInstance =
      Ogre::CompositorManager::getSingleton().addCompositor(
      _camera->OgreViewport(), "CameraDistortionMap/Default");
  this->dataPtr->lensDistortionInstance->setEnabled(true);

  // create the distortion map texture for the distortion instance
  std::string texName = _camera->Name() + "_distortionTex";
  Ogre::TexturePtr renderTexture =
      Ogre::TextureManager::getSingleton().createManual(
          texName,
          "General",
          Ogre::TEX_TYPE_2D,
          this->dataPtr->distortionTexWidth,
          this->dataPtr->distortionTexHeight,
          0,
          Ogre::PF_FLOAT32_RGB,
          Ogre::TU_STATIC_WRITE_ONLY);
  Ogre::HardwarePixelBufferSharedPtr pixelBuffer = renderTexture->getBuffer();

  // fill the distortion map, while interpolating to fill dead pixels
  pixelBuffer->lock(Ogre::HardwareBuffer::HBL_NORMAL);
  const Ogre::PixelBox &pixelBox = pixelBuffer->getCurrentLock();
  float *pDest = static_cast<float *>(pixelBox.data);
  for (unsigned int i = 0; i < this->dataPtr->distortionTexHeight; ++i)
  {
    for (unsigned int j = 0; j < this->dataPtr->distortionTexWidth; ++j)
    {
      math::Vector2d vec =
          this->dataPtr->distortionMap[i*this->dataPtr->distortionTexWidth+j];

      // perform interpolation on-the-fly:
      // check for empty mapping within the region and correct it by
      // interpolating four neighboring distortion map values.

      if (vec.x < -0.5 && vec.y < -0.5)
      {
        math::Vector2d left = this->GetDistortionMapValueClamped(j-1, i);
        math::Vector2d right = this->GetDistortionMapValueClamped(j+1, i);
        math::Vector2d bottom = this->GetDistortionMapValueClamped(j, i+1);
        math::Vector2d top = this->GetDistortionMapValueClamped(j, i-1);

        math::Vector2d top_left = this->GetDistortionMapValueClamped(j-1, i-1);
        math::Vector2d top_right = this->GetDistortionMapValueClamped(j+1, i-1);
        math::Vector2d bot_left = this->GetDistortionMapValueClamped(j-1, i+1);
        math::Vector2d bot_right = this->GetDistortionMapValueClamped(j+1, i+1);


        math::Vector2d interpolated;
        double divisor = 0;
        if (right.x > -0.5)
        {
          divisor++;
          interpolated += right;
        }
        if (left.x > -0.5)
        {
          divisor++;
          interpolated += left;
        }
        if (top.x > -0.5)
        {
          divisor++;
          interpolated += top;
        }
        if (bottom.x > -0.5)
        {
          divisor++;
          interpolated += bottom;
        }

        if (bot_right.x > -0.5)
        {
          divisor += 0.707;
          interpolated += bot_right * 0.707;
        }
        if (bot_left.x > -0.5)
        {
          divisor += 0.707;
          interpolated += bot_left * 0.707;
        }
        if (top_right.x > -0.5)
        {
          divisor += 0.707;
          interpolated += top_right * 0.707;
        }
        if (top_left.x > -0.5)
        {
          divisor += 0.707;
          interpolated += top_left * 0.707;
        }

        if (divisor > 0.5)
        {
          interpolated /= divisor;
        }
        *pDest++ = interpolated.x;
        *pDest++ = interpolated.y;
      }
      else
      {
        *pDest++ = vec.x;
        *pDest++ = vec.y;
      }

      *pDest++ = 0;  // Z-coordinate
    }
  }
  pixelBuffer->unlock();

  if (this->dataPtr->distortionCrop)
  {
    // I believe that if not used with a square distortion texture, this
    // calculation will result in stretching of the final output image.
    math::Vector2d boundA = this->Distort(math::Vector2d(0, 0),
        this->dataPtr->lensCenter,
        this->dataPtr->k1, this->dataPtr->k2, this->dataPtr->k3,
        this->dataPtr->p1, this->dataPtr->p2);
    math::Vector2d boundB = this->Distort(math::Vector2d(1, 1),
        this->dataPtr->lensCenter,
        this->dataPtr->k1, this->dataPtr->k2, this->dataPtr->k3,
        this->dataPtr->p1, this->dataPtr->p2);
    this->dataPtr->distortionScale = boundB - boundA;
    Ogre::GpuProgramParametersSharedPtr params =
        distMat->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
    params->setNamedConstant("scale",
        Ogre::Vector3(1.0/this->dataPtr->distortionScale.x,
        1.0/this->dataPtr->distortionScale.y, 1.0));
  }

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
bool Distortion::GetCrop() const
{
  return this->dataPtr->distortionCrop;
}

//////////////////////////////////////////////////
math::Vector2d Distortion::GetCenter() const
{
  return this->dataPtr->lensCenter;
}
