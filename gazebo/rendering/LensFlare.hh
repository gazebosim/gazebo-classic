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

#include <sdf/sdf.hh>
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
    /// \brief Camera lens flare compositor.
    class GZ_RENDERING_VISIBLE LensFlare
    {
      /// \brief Constructor
      public: LensFlare();

      /// \brief Destructor
      public: virtual ~LensFlare();

      /// \brief Set the camera which lensFlare will be applied to.
      /// \param[in] _camera Camera to be distorted
      public: void SetCamera(CameraPtr _camera);

      /// \brief Update function to search light source
      private: void Update();

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<LensFlarePrivate> dataPtr;
    };
    /// \}
  }
}
#endif
