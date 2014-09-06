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

#ifndef _GAZEBO_RENDERING_DISTORTION_HH_
#define _GAZEBO_RENDERING_DISTORTION_HH_

#include <sdf/sdf.hh>
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  /// \ingroup gazebo_rendering
  /// \brief Rendering namespace
  namespace rendering
  {
     class DistortionPrivate;

    /// \addtogroup gazebo_rendering Rendering
    /// \{

    /// \class Distortion Distortion.hh rendering/rendering.hh
    /// \brief Camera distortion based on Brown's model. Note that the current
    /// implementation has the limitation that it actually applies undistortion.
    /// So the distortion coefficients needs to be tweaked to emulate the
    /// desired barrel or pincushion distortions.
    class GAZEBO_VISIBLE Distortion
    {
      /// \brief Constructor
      public: Distortion();

      /// \brief Destructor
      public: virtual ~Distortion();

      /// \brief Load the camera with a set of parmeters
      /// \param[in] _sdf The SDF camera info
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief  Set the camera which distortion will be applied to.
      /// \param[in] _camera Camera to be distorted
      public: void SetCamera(CameraPtr _camera);

      /// \brief Finalize distortion.
      public: void Fini();

      /// \brief Distortion SDF values.
      protected: sdf::ElementPtr sdf;

      /// \internal
      /// \brief Pointer to private data.
      private: DistortionPrivate *dataPtr;
    };
    /// \}
  }
}
#endif
