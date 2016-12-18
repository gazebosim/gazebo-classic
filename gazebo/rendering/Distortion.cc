/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/Distortion.hh"

using namespace gazebo;
using namespace rendering;

namespace gazebo
{
  namespace rendering
  {
    /// \brief Private data for the Distortion class
    class DistortionPrivate
    {
      /// \brief Radial distortion coefficient k1.
      public: double k1 = 0;

      /// \brief Radial distortion coefficient k2.
      public: double k2 = 0;

      /// \brief Radial distortion coefficient k3.
      public: double k3 = 0;

      /// \brief Tangential distortion coefficient p1.
      public: double p1 = 0;

      /// \brief Tangential distortion coefficient p2.
      public: double p2 = 0;

      /// \brief Lens center used for distortion
      public: ignition::math::Vector2d lensCenter = {0.5, 0.5};

      /// \brief Scale applied to distorted image.
      public: ignition::math::Vector2d distortionScale = {1.0, 1.0};

      /// \brief True if the distorted image will be cropped to remove the
      /// black pixels at the corners of the image.
      public: bool distortionCrop = true;

      /// \brief Lens distortion compositor
      public: Ogre::CompositorInstance *lensDistortionInstance;

      /// \brief Connection for the pre render event.
      public: event::ConnectionPtr preRenderConnection;

      /// \brief Mapping of distorted to undistorted normalized pixels
      public: std::vector<ignition::math::Vector2d> distortionMap;
    };
  }
}
//////////////////////////////////////////////////
Distortion::Distortion()
  : dataPtr(new DistortionPrivate)
{
}

//////////////////////////////////////////////////
Distortion::~Distortion()
{
}

//////////////////////////////////////////////////
void Distortion::Load(sdf::ElementPtr _sdf)
{
  this->dataPtr->k1 = _sdf->Get<double>("k1");
  this->dataPtr->k2 = _sdf->Get<double>("k2");
  this->dataPtr->k3 = _sdf->Get<double>("k3");
  this->dataPtr->p1 = _sdf->Get<double>("p1");
  this->dataPtr->p2 = _sdf->Get<double>("p2");
  this->dataPtr->lensCenter = _sdf->Get<ignition::math::Vector2d>("center");

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
  unsigned int texSide = _camera->ImageHeight() > _camera->ImageWidth() ?
      _camera->ImageHeight() : _camera->ImageWidth();
  unsigned int texWidth = texSide;
  unsigned int texHeight = texSide;
  unsigned int imageSize = texWidth * texHeight;

  this->dataPtr->distortionMap.resize(imageSize);
  for (unsigned int i = 0; i < this->dataPtr->distortionMap.size(); ++i)
    this->dataPtr->distortionMap[i] = -1;

  double incrU = 1.0 / texWidth;
  double incrV = 1.0 / texHeight;

  // obtain bounds of the distorted image points.
  ignition::math::Vector2d boundA = this->Distort(
      ignition::math::Vector2d::Zero,
      this->dataPtr->lensCenter,
      this->dataPtr->k1, this->dataPtr->k2, this->dataPtr->k3,
      this->dataPtr->p1, this->dataPtr->p2);

  ignition::math::Vector2d boundB = this->Distort(ignition::math::Vector2d::One,
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
      ignition::math::Vector2d uv(u, v);
      ignition::math::Vector2d out =
        this->Distort(uv, this->dataPtr->lensCenter,
            this->dataPtr->k1, this->dataPtr->k2, this->dataPtr->k3,
            this->dataPtr->p1, this->dataPtr->p2);

      // compute the index in the distortion map
      unsigned int idxU = out.X() * texWidth;
      unsigned int idxV = out.Y() * texHeight;
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
  unsigned int boundAIdxU = boundA.X() * texWidth;
  unsigned int boundAIdxV = boundA.Y() * texHeight;
  unsigned int boundBIdxU = boundB.X() * texWidth;
  unsigned int boundBIdxV = boundB.Y() * texHeight;
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
      if (this->dataPtr->distortionMap[mapIdx] ==
          ignition::math::Vector2d(-1, -1))
      {
        ignition::math::Vector2d interpolate(0, 0);
        int sampleSize = 0;
        // left
        if ((boundAIdxU + j) != 0 &&
            !(this->dataPtr->distortionMap[mapIdx-1] ==
              ignition::math::Vector2d(-1, -1)))
        {
          interpolate += this->dataPtr->distortionMap[mapIdx-1];
          sampleSize++;
        }
        // right
        if ((boundAIdxU + j+1) < texWidth &&
            !(this->dataPtr->distortionMap[mapIdx+1] ==
              ignition::math::Vector2d(-1, -1)))
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
              == ignition::math::Vector2d(-1, -1)))
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
              == ignition::math::Vector2d(-1, -1)))
          {
            interpolate += this->dataPtr->distortionMap[mapIdx+1];
            sampleSize++;
          }
        }
        interpolate.X() = interpolate.X() / sampleSize;
        interpolate.Y() = interpolate.Y() / sampleSize;
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
      _camera->OgreViewport(), "CameraDistortionMap/Default");
  this->dataPtr->lensDistortionInstance->setEnabled(true);

  // create the distortion map texture
  std::string texName = _camera->Name() + "_distortionTex";
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
      ignition::math::Vector2d vec =
          this->dataPtr->distortionMap[i*texWidth+j];
      *pDest++ = vec.X();
      *pDest++ = vec.Y();
      *pDest++ = 0;
    }
  }
  pixelBuffer->unlock();

  // pass a scale param to the pixel shader for scaling the texture in order to
  // remove black border.
  Ogre::GpuProgramParametersSharedPtr params =
      distMat->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
  params->setNamedConstant("scale",
      Ogre::Vector3(this->dataPtr->distortionScale.X(),
      this->dataPtr->distortionScale.Y(), 1.0));

  // set up the distortion map texture to be used in the pixel shader.
  distMat->getTechnique(0)->getPass(0)->createTextureUnitState(texName, 1);
}

//////////////////////////////////////////////////
math::Vector2d Distortion::Distort(const math::Vector2d &_in,
    const math::Vector2d &_center, double _k1, double _k2, double _k3,
    double _p1, double _p2)
{
  return Distort(_in.Ign(), _center.Ign(), _k1, _k2, _k3, _p1, _p2);
}

//////////////////////////////////////////////////
ignition::math::Vector2d Distortion::Distort(
    const ignition::math::Vector2d &_in,
    const ignition::math::Vector2d &_center, double _k1, double _k2, double _k3,
    double _p1, double _p2)
{
  // apply Brown's distortion model, see
  // http://en.wikipedia.org/wiki/Distortion_%28optics%29#Software_correction

  ignition::math::Vector2d normalized2d = _in - _center;
  ignition::math::Vector3d normalized(normalized2d.X(), normalized2d.Y(), 0);
  double rSq = normalized.X() * normalized.X() +
               normalized.Y() * normalized.Y();

  // radial
  ignition::math::Vector3d dist = normalized * (1.0 +
      _k1 * rSq +
      _k2 * rSq * rSq +
      _k3 * rSq * rSq * rSq);

  // tangential
  dist.X() += _p2 * (rSq + 2 * (normalized.X()*normalized.X())) +
      2 * _p1 * normalized.X() * normalized.Y();
  dist.Y() += _p1 * (rSq + 2 * (normalized.Y()*normalized.Y())) +
      2 * _p2 * normalized.X() * normalized.Y();

  return _center + ignition::math::Vector2d(dist.X(), dist.Y());
}

//////////////////////////////////////////////////
void Distortion::SetCrop(const bool _crop)
{
  this->dataPtr->distortionCrop = _crop;
}

//////////////////////////////////////////////////
bool Distortion::Crop() const
{
  return this->dataPtr->distortionCrop;
}

//////////////////////////////////////////////////
double Distortion::GetK1() const
{
  return this->K1();
}

//////////////////////////////////////////////////
double Distortion::K1() const
{
  return this->dataPtr->k1;
}

//////////////////////////////////////////////////
double Distortion::GetK2() const
{
  return this->K2();
}

//////////////////////////////////////////////////
double Distortion::K2() const
{
  return this->dataPtr->k2;
}

//////////////////////////////////////////////////
double Distortion::GetK3() const
{
  return this->K3();
}

//////////////////////////////////////////////////
double Distortion::K3() const
{
  return this->dataPtr->k3;
}

//////////////////////////////////////////////////
double Distortion::GetP1() const
{
  return this->P1();
}

//////////////////////////////////////////////////
double Distortion::P1() const
{
  return this->dataPtr->p1;
}

//////////////////////////////////////////////////
double Distortion::GetP2() const
{
  return this->P2();
}

//////////////////////////////////////////////////
double Distortion::P2() const
{
  return this->dataPtr->p2;
}

//////////////////////////////////////////////////
math::Vector2d Distortion::GetCenter() const
{
  return this->Center();
}

//////////////////////////////////////////////////
ignition::math::Vector2d Distortion::Center() const
{
  return this->dataPtr->lensCenter;
}
