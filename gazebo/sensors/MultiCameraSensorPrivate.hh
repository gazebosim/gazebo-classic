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
#ifndef _GAZEBO_SENSORS_MULTICAMERA_SENSOR_PRIVATE_HH_
#define _GAZEBO_SENSORS_MULTICAMERA_SENSOR_PRIVATE_HH_

#include <vector>
#include <mutex>
#include <limits>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/sensors/Sensor.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \internal
    /// \brief Multicamera sensor private data.
    class MultiCameraSensorPrivate
    {
      /// \brief All the cameras.
      public: std::vector<rendering::CameraPtr> cameras;

      /// \brief Mutex to protect the cameras list.
      public: mutable std::mutex cameraMutex;

      /// \brief Publishes messages of type msgs::ImagesStamped.
      public: transport::PublisherPtr imagePub;

      /// \brief The images msg.
      public: msgs::ImagesStamped msg;

      /// \brief True if the sensor was rendered.
      public: bool rendered;

      /// \brief True if the sensor needs a rendering
      public: bool renderNeeded = false;

      /// \brief Rendering sensor extension class for ABI incompatible changes
      public: std::shared_ptr<RenderingSensorExt> extension;
    };
  }
}
#endif
