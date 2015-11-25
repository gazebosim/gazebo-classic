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
#ifndef _GAZEBO_ALTIMETER_SENSOR_PRIVATE_HH_
#define _GAZEBO_ALTIMETER_SENSOR_PRIVATE_HH_

#include <mutex>
#include <string>

#include "gazebo/sensors/SensorPrivate.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/msgs/msgs.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \internal
    /// \brief AltimeterSensor private data
    class AltimeterSensorPrivate : public SensorProtected
    {
      /// \brief Mutex to protect reads and writes.
      public: mutable std::mutex mutex;

      /// \brief Altimeter data publisher.
      public: transport::PublisherPtr altPub;

      /// \brief Parent link of this sensor.
      public: physics::LinkPtr parentLink;

      /// \brief Stores most recent altimeter sensor data.
      public: msgs::Altimeter altMsg;
    };
  }
}
#endif
