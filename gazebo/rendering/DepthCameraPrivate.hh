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
    // Forward declare private data.
    class DepthCameraPrivate;
    class ReflectanceRenderTargetListener;
    class ReflectanceMaterialListener;
    class ReflectanceMaterialSwitcher;

    // typedefs that are used only in this class
    using ReflectanceRenderTargetListenerPtr =
        std::shared_ptr<ReflectanceRenderTargetListener>;
    using ReflectanceMaterialListenerPtr =
        std::shared_ptr<ReflectanceMaterialListener>;
    using ReflectanceMaterialSwitcherPtr =
        std::shared_ptr<ReflectanceMaterialSwitcher>;

    /// \internal
    /// \brief Private data for the DepthCameraPrivate class
    class DepthCameraPrivate
    {
      /// \brief The depth buffer
      public: float *depthBuffer = nullptr;

      /// \brief The depth material
      public: Ogre::Material *depthMaterial = nullptr;

      /// \brief True to generate point clouds
      public: bool outputPoints;

      /// \brief True to generate reflectance
      public: bool outputReflectance;

      /// \brief True to generate normals
      public: bool outputNormals;

      /// \brief Point cloud data buffer
      public: float *pcdBuffer = nullptr;

      /// \brief reflectance data buffer
      public: float *reflectanceBuffer = nullptr;

      /// \brief Point cloud data buffer
      public: float *normalsBuffer = nullptr;

      /// \brief Point cloud view port
      public: Ogre::Viewport *pcdViewport = nullptr;

      /// \brief Point cloud view port
      public: Ogre::Viewport *normalsViewport = nullptr;

      /// \brief reflectance view port
      public: Ogre::Viewport *reflectanceViewport = nullptr;

      /// \brief Point cloud material
      public: Ogre::Material *pcdMaterial = nullptr;

      /// \brief reflectance material
      public: Ogre::Material *reflectanceMaterial = nullptr;

      /// \brief Point cloud material
      public: Ogre::Material *normalsMaterial = nullptr;

      /// \brief Point cloud texture
      public: Ogre::Texture *pcdTexture = nullptr;

      /// \brief reflectance texture
      public: Ogre::Texture *reflectanceTextures = nullptr;

      /// \brief Point cloud texture
      public: Ogre::RenderTarget *pcdTarget = nullptr;

      /// \brief reflectance texture
      public: Ogre::RenderTarget *reflectanceTarget = nullptr;

      /// \brief Pointer to reflectance material switcher.
      public: ReflectanceMaterialSwitcherPtr reflectanceMaterialSwitcher;

      /// \brief normals texture
      public: Ogre::Texture *normalsTextures = nullptr;

      /// \brief Point cloud texture
      public: Ogre::RenderTarget *normalsTarget = nullptr;

      /// \brief Event used to signal rgb point cloud data
      public: event::EventT<void(const float *, unsigned int, unsigned int,
                   unsigned int, const std::string &)> newRGBPointCloud;

      /// \brief Event used to signal depth data
      public: event::EventT<void(const float *, unsigned int, unsigned int,
                   unsigned int, const std::string &)> newDepthFrame;

      /// \brief Event used to signal reflectance data
      public: event::EventT<void(const float *, unsigned int, unsigned int,
                  unsigned int, const std::string &)> newReflectanceFrame;

      /// \brief Event used to signal normals point cloud data
      public: event::EventT<void(const float *, unsigned int, unsigned int,
                  unsigned int, const std::string &)> newNormalsPointCloud;
    };
  }
}
#endif
