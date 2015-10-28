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
#ifndef _GAZEBO_WIRELESS_TRANSCEIVER_HH_
#define _GAZEBO_WIRELESS_TRANSCEIVER_HH_

#include <string>
#include <ignition/math/Pose3.hh>

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/sensors/Sensor.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \addtogroup gazebo_sensors
    /// \{

    /// \class WirelessTransceiver WirelessTransceiver.hh sensors/sensors.hh
    /// \brief Sensor class for receiving wireless signals.
    class GAZEBO_VISIBLE WirelessTransceiver: public Sensor
    {
      /// \brief Constructor
      public: WirelessTransceiver();

      /// \brief Constructor
      public: ~WirelessTransceiver();

      // Documentation inherited
      public: virtual std::string GetTopic() const;

      // Documentation inherited
      public: virtual void Load(const std::string &_worldName);

      // Documentation inherited
      public: virtual void Init();

      // Documentation inherited
      public: virtual void Fini();

      /// \brief Returns the antenna's gain of the receiver (dBi).
      /// \return Antenna's gain of the receiver (dBi).
      public: double GetGain() const;

      /// \brief Returns the receiver power (dBm).
      /// \return Receiver power (dBm).
      public: double GetPower() const;

      /// \brief Publisher to publish propagation model data
      protected: transport::PublisherPtr pub;

      /// \brief Receiver's power (dBm).
      protected: double power;

      /// \brief Antenna's gain of the receiver (dBi).
      protected: double gain;

      /// \brief Parent entity which the sensor is attached to
      protected: boost::weak_ptr<physics::Link> parentEntity;

      /// \brief Sensor reference pose
      protected: ignition::math::Pose3d referencePose;
    };
    /// \}
  }
}
#endif
