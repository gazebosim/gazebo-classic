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
#ifndef _GAZEBO_SENSORS_LOGICAL_CAMERASENSOR_PRIVATE_HH_
#define _GAZEBO_SENSORS_LOGICAL_CAMERASENSOR_PRIVATE_HH_

#include <mutex>
#include <string>
#include <ignition/math/Frustum.hh>
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/Link.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \internal
    /// \brief Logical camera sensor private data.
    class LogicalCameraSensorPrivate
    {
      /// \brief Publisher of msgs::LogicalCameraImage messages.
      public: transport::PublisherPtr pub;

      /// \brief Camera frustum.
      public: ignition::math::Frustum frustum;

      /// \brief Pointer to the parent link.
      public: physics::LinkPtr parentLink;

      /// \brief Used to store and report the detected models.
      public: msgs::LogicalCameraImage msg;

      /// \brief Mutex to protect the msg.
      public: std::mutex mutex;

      /// \brief Name of the parent model.
      public: std::string modelName;
    };
  }
}
#endif
