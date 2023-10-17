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
#ifndef _GAZEBO_SENSORS_DEPTHCAMERASENSOR_PRIVATE_HH_
#define _GAZEBO_SENSORS_DEPTHCAMERASENSOR_PRIVATE_HH_

#include "gazebo/rendering/RenderTypes.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \internal
    /// \brief Depth camera sensor private data.
    class DepthCameraSensorPrivate
    {
      /// \brief Depth data buffer.
      public: float *depthBuffer = nullptr;

      /// \brief Reflectance data buffer.
      public: float *reflectanceBuffer = nullptr;

      /// \brief Local pointer to the depthCamera.
      public: rendering::DepthCameraPtr depthCamera;

      /// \brief Publisher of reflectance image messages.
      public: transport::PublisherPtr imageReflectancePub;
    };
  }
}
#endif
