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
#ifndef _GAZEBO_SENSORS_MAGNETOMETER_SENSOR_PRIVATE_HH_
#define _GAZEBO_SENSORS_MAGNETOMETER_SENSOR_PRIVATE_HH_

#include <mutex>

#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/msgs/msgs.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \internal
    /// \brief MagnetometerSensor private data.
    class MagnetometerSensorPrivate
    {
      /// \brief Mutex to protect reads and writes.
      public: mutable std::mutex mutex;

      /// \brief Magnetometer data publisher.
      public: transport::PublisherPtr magPub;

      /// \brief Parent link of this sensor.
      public: physics::LinkPtr parentLink;

      /// \brief Stores most recent magnetometer sensor data.
      public: msgs::Magnetometer magMsg;
    };
  }
}
#endif
