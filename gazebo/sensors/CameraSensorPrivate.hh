/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#ifndef _GAZEBO_CAMERASENSOR_PRIVATE_HH_
#define _GAZEBO_CAMERASENSOR_PRIVATE_HH_

#include "gazebo/sensors/SensorPrivate.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \internal
    /// \brief CameraSensor private data
    class CameraSensorPrivate : public SensorProtected
    {
      /// \brief Pointer to the camera.
      public: rendering::CameraPtr camera;

      /// \brief Publisher of image messages.
      public: transport::PublisherPtr imagePub;

      /// \brief True if the sensor was rendered.
      public: bool rendered;
    };
  }
}
#endif
