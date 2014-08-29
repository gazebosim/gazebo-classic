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
  std::string type = this->sdf->Get<std::string>("type");
  if (type == "barrel")
    this->dataPtr->distortionType = DistortionPrivate::BARREL;
  this->dataPtr->radialCoeff = this->sdf->Get<math::Vector3>("radial");
  this->dataPtr->tangentialCoeff = this->sdf->Get<math::Vector2d>("tangential");
  this->dataPtr->lensCenter = this->sdf->Get<math::Vector2d>("center");
  this->dataPtr->distortionCrop = this->sdf->Get<bool>("crop");
}

//////////////////////////////////////////////////
void Distortion::SetCamera(CameraPtr _camera)
{
  GZ_ASSERT(_camera, "Unable to apply distortion, camera is NULL");

  this->dataPtr->distortionScale.x = 1.0;
  this->dataPtr->distortionScale.y = 1.0;

  if (this->dataPtr->distortionCrop
      && this->dataPtr->distortionType == DistortionPrivate::BARREL)
  {
    double tlx = 0;
    double tly = 0;
    double brx = 0;
    double bry = 0;
    double dtl = GZ_DBL_MAX;
    double dbr = GZ_DBL_MAX;
    double incrU = 1.0 / (_camera->GetImageWidth());
    double incrV = 1.0 / (_camera->GetImageHeight());
    for (double u = 0; u < 1.0; u+=incrU)
    {
      for (double v = 0; v < 1.0; v+=incrV)
      {
        math::Vector2d texCoord(u, v);
        math::Vector2d normalized2d = texCoord - this->dataPtr->lensCenter;
        math::Vector3 normalized(normalized2d.x, normalized2d.y, 0);
        double rSq = normalized.x * normalized.x
            + normalized.y * normalized.y;
        math::Vector3 dist = normalized * ( 1.0 +
            this->dataPtr->radialCoeff.x * rSq +
            this->dataPtr->radialCoeff.y * rSq * rSq +
            this->dataPtr->radialCoeff.z * rSq * rSq * rSq);
        dist.x += this->dataPtr->tangentialCoeff.x
            * (rSq + 2 * (normalized.x*normalized.x)) +
            2 * this->dataPtr->tangentialCoeff.y
            * normalized.x * normalized.y;
        dist.y += this->dataPtr->tangentialCoeff.y
            * (rSq + 2 * (normalized.y*normalized.y)) +
            2 * this->dataPtr->tangentialCoeff.x
            * normalized.x * normalized.y;
        math::Vector2d out = this->dataPtr->lensCenter
            + math::Vector2d(dist.x, dist.y);
        double dx = out.x - 0.0;
        double dy = out.y - 0.0;
        if (dx >= 0 && dy >= 0 && (dx + dy) < dtl)
        {
          tlx = u;
          tly = v;
          dtl = dx + dy;
        }
        dx = out.x - 1.0;
        dy = out.y - 1.0;
        if (dx <= 0 && dy <= 0 && (fabs(dx) + fabs(dy)) < dbr)
        {
          brx = u;
          bry = v;
          dbr = (fabs(dx) + fabs(dy));
        }
      }
    }
    this->dataPtr->distortionScale.x = brx - tlx;
    this->dataPtr->distortionScale.y = bry - tly;
  }

  Ogre::MaterialPtr distMat =
      Ogre::MaterialManager::getSingleton().getByName(
      "Gazebo/CameraDistortion");
  Ogre::GpuProgramParametersSharedPtr params =
      distMat->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
  params->setNamedConstant("radial",
      static_cast<Ogre::Vector3>(
      Conversions::Convert(this->dataPtr->radialCoeff)));
  params->setNamedConstant("tangential",
      static_cast<Ogre::Vector3>(
      Ogre::Vector3(this->dataPtr->tangentialCoeff.x,
      this->dataPtr->tangentialCoeff.y, 0.0)));
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
