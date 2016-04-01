/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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

#ifndef _GAZEBO_RENDERING_DEPTHCAMERA_PRIVATE_HH_
#define _GAZEBO_RENDERING_DEPTHCAMERA_PRIVATE_HH_

#include <string>

#include "gazebo/common/Event.hh"

#include "gazebo/rendering/Camera.hh"

namespace Ogre
{
  class Material;
  class RenderTarget;
  class Texture;
  class Viewport;
}

namespace gazebo
{
  namespace rendering
  {
    /// \internal
    /// \brief Private data for the DepthCameraPrivate class
    class DepthCameraPrivate
    {
      /// \brief The depth buffer
      public: float *depthBuffer;

      /// \brief The depth material
      public: Ogre::Material *depthMaterial;

      /// \brief True to generate point clouds
      public: bool outputPoints;

      /// \brief Point cloud data buffer
      public: float *pcdBuffer;

      /// \brief Point cloud view port
      public: Ogre::Viewport *pcdViewport;

      /// \brief Point cloud material
      public: Ogre::Material *pcdMaterial;

      /// \brief Point cloud texture
      public: Ogre::Texture *pcdTexture;

      /// \brief Point cloud texture
      public: Ogre::RenderTarget *pcdTarget;

      /// \brief Event used to signal rgb point cloud data
      public: event::EventT<void(const float *, unsigned int, unsigned int,
                   unsigned int, const std::string &)> newRGBPointCloud;

      /// \brief Event used to signal depth data
      public: event::EventT<void(const float *, unsigned int, unsigned int,
                   unsigned int, const std::string &)> newDepthFrame;
    };
  }
}
#endif
