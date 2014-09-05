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
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/DistortionPrivate.hh"
#include "gazebo/rendering/Distortion.hh"

using namespace gazebo;
using namespace rendering;

//////////////////////////////////////////////////
Distortion::Distortion()
  : dataPtr(new DistortionPrivate)
{
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
  this->dataPtr->distortionCrop = this->sdf->Get<bool>("crop");
}

//////////////////////////////////////////////////
void Distortion::SetCamera(CameraPtr _camera)
{
  GZ_ASSERT(_camera, "Unable to apply distortion, camera is NULL");

  this->dataPtr->distortionScale.x = 1.0;
  this->dataPtr->distortionScale.y = 1.0;

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
        double dx = out.x;
        double dy = out.y;

        // crop by finding corners that would give the largest resolution.
        // One way to compute this is just to look at (dx + dy) where
        // dx is the center-to-u distance, and
        // dy is the center-to-v distance
        if (dx >= 0 && dy >= 0 && (dx + dy) < dtr)
        {
          trX = u;
          trY = v;
          dtr = dx + dy;
          std::cerr << " setting dtr " << u << " " << v << std::endl;
          std::cerr << " dx dy " <<
              dx << " " << dy << std::endl;
        }
        dx = out.x - 1.0;
        dy = out.y - 1.0;
        if (dx <= 0 && dy <= 0 && (fabs(dx) + fabs(dy)) < dbl)
        {
          blX = u;
          blY = v;
          dbl = (fabs(dx) + fabs(dy));
          std::cerr << " setting dbl " << u << " " << v << std::endl;
          std::cerr << " dx dy " <<
              dx << " " << dy << std::endl;
        }
      }
    }
    std::cerr << " tr bl " <<
        trX << " " << trY << " " << blX << " " << blY << std::endl;

    this->dataPtr->distortionScale.x = blX - trX;
    this->dataPtr->distortionScale.y = blY - trY;
  }

  std::cerr << " this->dataPtr->distortionScale " <<
      this->dataPtr->distortionScale << std::endl;

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
}

//////////////////////////////////////////////////
void Distortion::Fini()
{
}
