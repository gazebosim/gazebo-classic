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
#ifndef _GAZEBO_WIRELESS_TRANSCEIVER_PRIVATE_HH_
#define _GAZEBO_WIRELESS_TRANSCEIVER_PRIVATE_HH_

#include <ignition/math/Pose3.hh>

#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/sensors/SensorPrivate.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \internal
    /// \brief Wireless transceiver private data
    class WirelessTransceiverPrivate : public SensorProtected
    {
      public: WirelessTransceiverPrivate() : power(14.5), gain(2.5) {}

      /// \brief Publisher to publish propagation model data
      public: transport::PublisherPtr pub;

      /// \brief Receiver's power (dBm).
      public: double power;

      /// \brief Antenna's gain of the receiver (dBi).
      public: double gain;

      /// \brief Parent entity which the sensor is attached to
      public: boost::weak_ptr<physics::Link> parentEntity;

      /// \brief Sensor reference pose
      public: ignition::math::Pose3d referencePose;
    };
  }
}
#endif
