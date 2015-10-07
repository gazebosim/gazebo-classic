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
#ifndef _GAZEBO_RENDERING_WIDE_ANGLE_CAMERA_CAMERA_SENSOR_PRIVATE_HH_
#define _GAZEBO_RENDERING_WIDE_ANGLE_CAMERA_CAMERA_SENSOR_PRIVATE_HH_

#include <mutex>
#include <queue>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/msgs/MessageTypes.hh"
#include "gazebo/util/system.hh"


namespace gazebo
{
  namespace sensors
  {
    /// \brief Private data for the WideAngleCameraSensor class
    class GZ_RENDERING_VISIBLE WideAngleCameraSensorPrivate
    {
      /// \brief Publisher of lens info messages
      public: transport::PublisherPtr lensPub;

      /// \brief Subscriber to lens control messages
      public: transport::SubscriberPtr lensSub;

      /// \brief Mutex to lock when receiving or sending lens message
      public: std::mutex lensCmdMutex;

      /// \brief Horizontal FOV updates to be set in rendering thread
      public: std::queue<double> hfovCmdQueue;
    };
  }
}
#endif
