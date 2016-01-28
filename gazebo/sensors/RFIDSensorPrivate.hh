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
#ifndef _GAZEBO_SENSORS_RFIDSENSOR_PRIVATE_HH_
#define _GAZEBO_SENSORS_RFIDSENSOR_PRIVATE_HH_

#include <vector>

#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/sensors/RFIDTag.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \internal
    /// \brief RFID sensor private data.
    class RFIDSensorPrivate
    {
      /// \brief Parent entity
      public: physics::EntityPtr entity;

      /// \brief Publisher for RFID pose messages.
      public: transport::PublisherPtr scanPub;

      /// \brief All the RFID tags.
      public: std::vector<RFIDTag*> tags;
    };
  }
}
#endif
