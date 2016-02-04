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
#ifndef _GAZEBO_SENSORS_SENSOR_PRIVATE_HH_
#define _GAZEBO_SENSORS_SENSOR_PRIVATE_HH_

#include <mutex>
#include <sdf/sdf.hh>

#include "gazebo/rendering/RenderTypes.hh"

#include "gazebo/common/Event.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \internal
    /// \brief Sensor private data.
    class SensorPrivate
    {
      /// \brief Mutex to protect resetting lastUpdateTime.
      public: std::mutex mutexLastUpdateTime;

      /// \brief Event triggered when a sensor is updated.
      public: event::EventT<void()> updated;

      /// \brief Publish sensor data.
      public: transport::PublisherPtr sensorPub;

      /// \brief The category of the sensor.
      public: SensorCategory category;

      /// \brief Keep track how much the update has been delayed.
      public: common::Time updateDelay;

      /// \brief The sensors unique ID.
      public: uint32_t id;

      /// \brief An SDF pointer that allows us to only read the sensor.sdf
      /// file once, which in turns limits disk reads.
      public: static sdf::ElementPtr sdfSensor;
    };
    /// \}
  }
}
#endif
