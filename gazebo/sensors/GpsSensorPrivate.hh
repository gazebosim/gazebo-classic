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
#ifndef _GAZEBO_GPSSENSOR_PRIVATE_HH_
#define _GAZEBO_GPSSENSOR_PRIVATE_HH_

#include <string>

#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/sensors/SensorPrivate.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \internal
    /// \brief GPS sensor private data.
    class GpsSensorPrivate : public SensorProtected
    {
      /// \brief GPS data publisher.
      public: transport::PublisherPtr gpsPub;

      /// \brief Topic name for GPS data publisher.
      public: std::string topicName;

      /// \brief Parent link of this sensor.
      public: physics::LinkPtr parentLink;

      /// \brief Pointer to SphericalCoordinates converter.
      public: common::SphericalCoordinatesPtr sphericalCoordinates;

      /// \brief Stores most recent GPS sensor data.
      public: msgs::GPS lastGpsMsg;
    };
  }
}
#endif
