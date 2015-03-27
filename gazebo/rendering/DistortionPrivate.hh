/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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

#include <vector>

#include "gazebo/math/Vector2d.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \brief Private data for the Distortion class
    class DistortionPrivate
    {
      /// \brief Radial distortion coefficient k1.
      public: double k1;

      /// \brief Radial distortion coefficient k2.
      public: double k2;

      /// \brief Radial distortion coefficient k3.
      public: double k3;

      /// \brief Tangential distortion coefficient p1.
      public: double p1;

      /// \brief Tangential distortion coefficient p2.
      public: double p2;

      /// \brief Lens center used for distortion
      public: math::Vector2d lensCenter;

      /// \brief Scale applied to distorted image.
      public: math::Vector2d distortionScale;

      /// \brief True if the distorted image will be cropped to remove the
      /// black pixels at the corners of the image.
      public: bool distortionCrop;

      /// \brief Lens distortion compositor
      public: Ogre::CompositorInstance *lensDistortionInstance;

      /// \brief Connection for the pre render event.
      public: event::ConnectionPtr preRenderConnection;

      /// \brief Mapping of distorted to undistorted normalized pixels
      public: std::vector<math::Vector2d> distortionMap;
    };
  }
}
#endif
