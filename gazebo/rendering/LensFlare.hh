/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#ifndef GAZEBO_RENDERING_LENSFLARE_HH_
#define GAZEBO_RENDERING_LENSFLARE_HH_

#include <memory>

#include "gazebo/msgs/msgs.hh"

#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  /// \ingroup gazebo_rendering
  /// \brief Rendering namespace
  namespace rendering
  {
     class LensFlarePrivate;

    /// \addtogroup gazebo_rendering Rendering
    /// \{

    /// \class LensFlare LensFlare.hh rendering/rendering.hh
    /// \brief Camera lens flare compositor. This lens flare effect does not
    /// perform any depth checking so if the directional light is occluded by an
    /// object in the scene, lens flare will pass through the object. The lens
    /// flare's color is set by the shaders and not exposed through an API in
    /// this class.
    class GZ_RENDERING_VISIBLE LensFlare
    {
      /// \brief Constructor
      public: LensFlare();

      /// \brief Destructor
      public: virtual ~LensFlare();

      /// \brief Set the camera which lensFlare will be applied to.
      /// \param[in] _camera Camera to be distorted
      public: void SetCamera(CameraPtr _camera);

      /// \brief Set the scale of lens flare. Must be greater than 0.
      /// \param[in] _scale Scale of lens flare
      public: void SetScale(const double _scale);

      /// \brief Update function to search light source
      private: void Update();

      /// \brief Request callback
      /// \param[in] _msg The message data.
      private: void OnRequest(ConstRequestPtr &_msg);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<LensFlarePrivate> dataPtr;
    };
    /// \}
  }
}
#endif
