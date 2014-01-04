/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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
#ifndef _GAZEBO_RENDERING_DISTORTION_PRIVATE_HH_
#define _GAZEBO_RENDERING_DISTORTION_PRIVATE_HH_

#include "gazebo/math/Vector2d.hh"
#include "gazebo/math/Vector3.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \brief Private data for the Camera class
    class DistortionPrivate
    {
      /// \brief Which distortion type we support
      public: enum DistortionModelType
      {
        BARREL
      };

      /// \brief Type of lens distortion we're applying
      public: DistortionModelType distortionType;

      /// \brief Radial distortion coefficients.
      public: math::Vector3 radialCoeff;

      /// \brief Tangential distortion coefficients.
      public: math::Vector2d tangentialCoeff;

      /// \brief Lens center used for distortion
      public: math::Vector2d lensCenter;

      /// \brief Scale applied to distorted image.
      public: math::Vector2d distortionScale;

      /// \brief True if the distorted image will be cropped to remove the
      /// black pixels at the corners of the image.
      public: bool distortionCrop;

      /// \brief Lens distortion compositor
      public: Ogre::CompositorInstance *lensDistortionInstance;
    };
  }
}
#endif
