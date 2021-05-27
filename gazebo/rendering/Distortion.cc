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

#include <ignition/math/Helpers.hh>

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
    class DistortionPrivate : public Ogre::CompositorInstance::Listener
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

      /// \brief Compositor name to be used for distortion
      public: std::string compositorName = "CameraDistortionMap/Default";

      /// \brief Scale applied to distorted image.
      public: ignition::math::Vector2d distortionScale = {1.0, 1.0};

      /// \brief True if the distorted image will be cropped to remove the
      /// black pixels at the corners of the image.
      public: bool distortionCrop = true;

      /// \brief Lens distortion compositor
      public: Ogre::CompositorInstance *lensDistortionInstance;

      /// \brief Ogre Material that contains the distortion shader
      public: Ogre::MaterialPtr distortionMaterial;

      /// \brief Connection for the pre render event.
      public: event::ConnectionPtr preRenderConnection;

      /// \brief Mapping of distorted to undistorted normalized pixels
      public: std::vector<ignition::math::Vector2d> distortionMap;

      /// \brief Width of distortion texture map
      public: unsigned int distortionTexWidth;

      /// \brief Height of distortion texture map
      public: unsigned int distortionTexHeight;

      // \brief Set scale parameter in shader before rendering frame
      public:
      virtual void notifyMaterialRender(Ogre::uint32 _passId,
                                        Ogre::MaterialPtr& _material)
      {
        // @todo Explore more efficent implementations as it is run every frame
        Ogre::GpuProgramParametersSharedPtr params =
            _material->getTechnique(0)->getPass(_passId)
                     ->getFragmentProgramParameters();
        params->setNamedConstant("scale",
            Ogre::Vector3(1.0/distortionScale.X(),
            1.0/distortionScale.Y(), 1.0));
      }
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

  this->dataPtr->distortionCrop = this->dataPtr->k1 < 0;

  const std::string compositorName = "ignition:compositor";
  if (_sdf->HasElement(compositorName))
  {
    this->dataPtr->compositorName = _sdf->Get<std::string>(compositorName);
  }
}

//////////////////////////////////////////////////
ignition::math::Vector2d
    Distortion::DistortionMapValueClamped(const int x, const int y) const
{
  if (x < 0 || x >= static_cast<int>(this->dataPtr->distortionTexWidth) ||
      y < 0 || y >= static_cast<int>(this->dataPtr->distortionTexHeight))
  {
    return ignition::math::Vector2d(-1, -1);
  }
  ignition::math::Vector2d res =
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

  // If no distortion is required, immediately return.
  if (ignition::math::equal(this->dataPtr->k1, 0.0) &&
      ignition::math::equal(this->dataPtr->k2, 0.0) &&
      ignition::math::equal(this->dataPtr->k3, 0.0) &&
      ignition::math::equal(this->dataPtr->p1, 0.0) &&
      ignition::math::equal(this->dataPtr->p2, 0.0))
  {
    return;
  }

  // seems to work best with a square distortion map texture
  unsigned int texSide = _camera->ImageHeight() > _camera->ImageWidth() ?
      _camera->ImageHeight() : _camera->ImageWidth();
  // calculate focal length from largest fov
  double fov = _camera->ImageHeight() > _camera->ImageWidth() ?
      _camera->VFOV().Radian() : _camera->HFOV().Radian();
  double focalLength = texSide/(2*tan(fov/2));
  this->dataPtr->distortionTexWidth = texSide - 1;
  this->dataPtr->distortionTexHeight = texSide - 1;
  unsigned int imageSize =
      this->dataPtr->distortionTexWidth * this->dataPtr->distortionTexHeight;
  double colStepSize = 1.0 / this->dataPtr->distortionTexWidth;
  double rowStepSize = 1.0 / this->dataPtr->distortionTexHeight;

  // initialize distortion map
  this->dataPtr->distortionMap.resize(imageSize);
  for (unsigned int i = 0; i < this->dataPtr->distortionMap.size(); ++i)
  {
    this->dataPtr->distortionMap[i] = -1;
  }

  ignition::math::Vector2d distortionCenterCoordinates(
      this->dataPtr->lensCenter.X() * this->dataPtr->distortionTexWidth,
      this->dataPtr->lensCenter.Y() * this->dataPtr->distortionTexWidth);

  // declare variables before the loop
  static auto unsetPixelVector =  ignition::math::Vector2d(-1, -1);
  ignition::math::Vector2d normalizedLocation,
      distortedLocation,
      newDistortedCoordinates,
      currDistortedCoordinates;
  unsigned int distortedIdx,
      distortedCol,
      distortedRow;
  double normalizedColLocation, normalizedRowLocation;

  // fill the distortion map
  for (unsigned int mapRow = 0; mapRow < this->dataPtr->distortionTexHeight; ++mapRow)
  {
    normalizedRowLocation = mapRow*rowStepSize;
    for (unsigned int mapCol = 0; mapCol < this->dataPtr->distortionTexWidth; ++mapCol)
    {
      normalizedColLocation = mapCol*colStepSize;

      normalizedLocation[0] = normalizedColLocation;
      normalizedLocation[1] = normalizedRowLocation;

      distortedLocation = this->Distort(
          normalizedLocation,
          this->dataPtr->lensCenter,
          this->dataPtr->k1, this->dataPtr->k2, this->dataPtr->k3,
          this->dataPtr->p1, this->dataPtr->p2,
          this->dataPtr->distortionTexWidth,
          focalLength);

      // compute the index in the distortion map
      distortedCol = distortedLocation.X() * this->dataPtr->distortionTexWidth;
      distortedRow = distortedLocation.Y() * this->dataPtr->distortionTexHeight;

      // Make sure the distorted pixel is within the texture dimensions
      if (distortedCol < this->dataPtr->distortionTexWidth &&
          distortedRow < this->dataPtr->distortionTexHeight)
      {
        distortedIdx = distortedRow * this->dataPtr->distortionTexWidth + distortedCol;

        // check if the index has already been set
        if (this->dataPtr->distortionMap[distortedIdx] != unsetPixelVector)
        {
          // grab current coordinates that map to this destination
          currDistortedCoordinates = this->dataPtr->distortionMap[distortedIdx] *
                                     this->dataPtr->distortionTexWidth;

          // grab new coordinates to map to
          newDistortedCoordinates[0] = mapCol;
          newDistortedCoordinates[1] = mapRow;

          // use the new mapping if it is closer to the center of the distortion
          if (newDistortedCoordinates.Distance(distortionCenterCoordinates) <
              currDistortedCoordinates.Distance(distortionCenterCoordinates))
          {
            this->dataPtr->distortionMap[distortedIdx] = normalizedLocation;
          }
        }
        else
        {
          this->dataPtr->distortionMap[distortedIdx] = normalizedLocation;
        }
      }
      // else: mapping is outside of the image bounds.
      // This is expected and normal to ensure
      // no black borders; carry on
    }
  }

  // set up the distortion instance
  this->dataPtr->distortionMaterial =
      Ogre::MaterialManager::getSingleton().getByName(
          "Gazebo/CameraDistortionMap");
  this->dataPtr->distortionMaterial =
      this->dataPtr->distortionMaterial->clone(
          "Gazebo/" + _camera->Name() + "_CameraDistortionMap");

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
          Ogre::PF_FLOAT32_RGB);
  Ogre::HardwarePixelBufferSharedPtr pixelBuffer = renderTexture->getBuffer();

  // fill the distortion map, while interpolating to fill dead pixels
  pixelBuffer->lock(Ogre::HardwareBuffer::HBL_NORMAL);
  const Ogre::PixelBox &pixelBox = pixelBuffer->getCurrentLock();

#if OGRE_VERSION_MAJOR > 1 || OGRE_VERSION_MINOR >= 11
  // Ogre 1.11 changed Ogre::PixelBox::data from void* to uchar*, hence
  // reinterpret_cast is required here. static_cast is not allowed between
  // pointers of unrelated types (see, for instance, Standard § 3.9.1
  // Fundamental types)
  float *pDest = reinterpret_cast<float *>(pixelBox.data);
#else
  float *pDest = static_cast<float *>(pixelBox.data);
#endif

  for (unsigned int i = 0; i < this->dataPtr->distortionTexHeight; ++i)
  {
    for (unsigned int j = 0; j < this->dataPtr->distortionTexWidth; ++j)
    {
      ignition::math::Vector2d vec =
          this->dataPtr->distortionMap[i*this->dataPtr->distortionTexWidth+j];

      // perform interpolation on-the-fly:
      // check for empty mapping within the region and correct it by
      // interpolating the eight neighboring distortion map values.

      if (vec.X() < -0.5 && vec.Y() < -0.5)
      {
        ignition::math::Vector2d left =
            this->DistortionMapValueClamped(j-1, i);
        ignition::math::Vector2d right =
            this->DistortionMapValueClamped(j+1, i);
        ignition::math::Vector2d bottom =
            this->DistortionMapValueClamped(j, i+1);
        ignition::math::Vector2d top =
            this->DistortionMapValueClamped(j, i-1);

        ignition::math::Vector2d topLeft =
            this->DistortionMapValueClamped(j-1, i-1);
        ignition::math::Vector2d topRight =
            this->DistortionMapValueClamped(j+1, i-1);
        ignition::math::Vector2d bottomLeft =
            this->DistortionMapValueClamped(j-1, i+1);
        ignition::math::Vector2d bottomRight =
            this->DistortionMapValueClamped(j+1, i+1);


        ignition::math::Vector2d interpolated;
        double divisor = 0;
        if (right.X() > -0.5)
        {
          divisor++;
          interpolated += right;
        }
        if (left.X() > -0.5)
        {
          divisor++;
          interpolated += left;
        }
        if (top.X() > -0.5)
        {
          divisor++;
          interpolated += top;
        }
        if (bottom.X() > -0.5)
        {
          divisor++;
          interpolated += bottom;
        }

        if (bottomRight.X() > -0.5)
        {
          divisor += 0.707;
          interpolated += bottomRight * 0.707;
        }
        if (bottomLeft.X() > -0.5)
        {
          divisor += 0.707;
          interpolated += bottomLeft * 0.707;
        }
        if (topRight.X() > -0.5)
        {
          divisor += 0.707;
          interpolated += topRight * 0.707;
        }
        if (topLeft.X() > -0.5)
        {
          divisor += 0.707;
          interpolated += topLeft * 0.707;
        }

        if (divisor > 0.5)
        {
          interpolated /= divisor;
        }
        *pDest++ = ignition::math::clamp(interpolated.X(), 0.0, 1.0);
        *pDest++ = ignition::math::clamp(interpolated.Y(), 0.0, 1.0);
      }
      else
      {
        *pDest++ = vec.X();
        *pDest++ = vec.Y();
      }

      // Z coordinate
      *pDest++ = 0;
    }
  }
  pixelBuffer->unlock();

  // set up the distortion map texture to be used in the pixel shader.
  this->dataPtr->distortionMaterial->getTechnique(0)->getPass(0)->
      createTextureUnitState(texName, 1);

  this->dataPtr->lensDistortionInstance =
      Ogre::CompositorManager::getSingleton().addCompositor(
      _camera->OgreViewport(), this->dataPtr->compositorName);
  this->dataPtr->lensDistortionInstance->getTechnique()->getOutputTargetPass()->
      getPass(0)->setMaterial(this->dataPtr->distortionMaterial);

  this->CalculateAndApplyDistortionScale();

  this->dataPtr->lensDistortionInstance->setEnabled(true);

  // Add callback to set scaling factor before rendering
  // See https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2963
  this->dataPtr->lensDistortionInstance->addListener(this->dataPtr.get());
}

//////////////////////////////////////////////////
void Distortion::CalculateAndApplyDistortionScale()
{
  if (this->dataPtr->distortionMaterial.isNull())
    return;

  // Scale up image if cropping enabled and valid
  if (this->dataPtr->distortionCrop && this->dataPtr->k1 < 0)
  {
    // I believe that if not used with a square distortion texture, this
    // calculation will result in stretching of the final output image.
    ignition::math::Vector2d boundA = this->Distort(
        ignition::math::Vector2d(0, 0),
        this->dataPtr->lensCenter,
        this->dataPtr->k1, this->dataPtr->k2, this->dataPtr->k3,
        this->dataPtr->p1, this->dataPtr->p2);
    ignition::math::Vector2d boundB = this->Distort(
        ignition::math::Vector2d(1, 1),
        this->dataPtr->lensCenter,
        this->dataPtr->k1, this->dataPtr->k2, this->dataPtr->k3,
        this->dataPtr->p1, this->dataPtr->p2);
    ignition::math::Vector2d newScale = boundB - boundA;
    // If distortionScale is extremely small, don't crop
    if (newScale.X() < 1e-7 || newScale.Y() < 1e-7)
    {
          gzerr << "Distortion model attempted to apply a scale parameter of ("
                << this->dataPtr->distortionScale.X() << ", "
                << this->dataPtr->distortionScale.Y()
                << ", which is invalid.\n";
    }
    else
      this->dataPtr->distortionScale = newScale;
  }
  // Otherwise no scaling
  else
    this->dataPtr->distortionScale = ignition::math::Vector2d(1, 1);
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
ignition::math::Vector2d Distortion::Distort(
    const ignition::math::Vector2d &_in,
    const ignition::math::Vector2d &_center, double _k1, double _k2, double _k3,
    double _p1, double _p2, unsigned int _width, double _f)
{
  // apply Brown's distortion model, see
  // http://en.wikipedia.org/wiki/Distortion_%28optics%29#Software_correction

  ignition::math::Vector2d normalized2d = (_in - _center)*(_width/_f);
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

  return ((_center*_width) + ignition::math::Vector2d(dist.X(), dist.Y())*_f)/_width;
}

//////////////////////////////////////////////////
void Distortion::SetCrop(const bool _crop)
{
  // Only update the distortion scale if the crop value is going to flip.
  if (this->dataPtr->distortionCrop != _crop)
  {
    this->dataPtr->distortionCrop = _crop;
    this->CalculateAndApplyDistortionScale();
  }
}

//////////////////////////////////////////////////
bool Distortion::Crop() const
{
  return this->dataPtr->distortionCrop;
}

//////////////////////////////////////////////////
double Distortion::K1() const
{
  return this->dataPtr->k1;
}

//////////////////////////////////////////////////
double Distortion::K2() const
{
  return this->dataPtr->k2;
}

//////////////////////////////////////////////////
double Distortion::K3() const
{
  return this->dataPtr->k3;
}

//////////////////////////////////////////////////
double Distortion::P1() const
{
  return this->dataPtr->p1;
}

//////////////////////////////////////////////////
double Distortion::P2() const
{
  return this->dataPtr->p2;
}

//////////////////////////////////////////////////
ignition::math::Vector2d Distortion::Center() const
{
  return this->dataPtr->lensCenter;
}
