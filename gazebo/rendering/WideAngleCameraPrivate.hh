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
#ifndef _GAZEBO_RENDERING_WIDE_ANGLE_CAMERA_CAMERA_PRIVATE_HH_
#define _GAZEBO_RENDERING_WIDE_ANGLE_CAMERA_CAMERA_PRIVATE_HH_

#include <mutex>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/util/system.hh"


// Forward declarations
namespace Ogre
{
  class Camera;
  class CompositorInstance;
  class RenderTarget;
  class Texture;
  class Viewport;
}

namespace gazebo
{
  namespace rendering
  {
    // Forward declarations
    class CameraLens;

    /// \brief Private data for the WideAngleCamera class
    class GZ_RENDERING_VISIBLE WideAngleCameraPrivate
    {
      /// \brief Environment texture size
      public: int envTextureSize;

      /// \brief Compositor used to render rectangle with attached cube map
      public: Ogre::CompositorInstance *cubeMapCompInstance;

      /// \brief A Set of 6 cameras,
      ///   each pointing in different direction with FOV of 90deg
      public: Ogre::Camera *envCameras[6];

      /// \brief Render targets for envCameras
      public: Ogre::RenderTarget *envRenderTargets[6];

      /// \brief Viewports for the render targets
      public: Ogre::Viewport *envViewports[6];

      /// \brief A single cube map texture
      public: Ogre::Texture *envCubeMapTexture;

      /// \brief Pointer to material, used for second rendering pass
      public: Ogre::MaterialPtr compMat;

      /// \brief Camera lens description
      public: CameraLens *lens;

      /// \brief Mutex to lock while rendering the world
      public: std::mutex renderMutex;

      /// \brief Mutex to lock while setting or reading camera properties
      public: std::mutex dataMutex;
    };
  }
}
#endif
