/*
 * Copyright 2013 Open Source Robotics Foundation
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
/* Desc: Wireless transceiver
 * Author: Carlos Agüero
 * Date: 03 July 2013
 */

#ifndef _WIRELESS_TRANSCEIVER_HH_
#define _WIRELESS_TRANSCEIVER_HH_

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/sensors/Sensor.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  /// \ingroup gazebo_sensors
  /// \brief Sensors namespace
  namespace sensors
  {
    /// \addtogroup gazebo_sensors
    /// \{

    /// \class WirelessTransceiver WirelessTransceiver.hh sensors/sensors.hh
    /// \brief Sensor class for receiving wireless signals.
    class WirelessTransceiver: public Sensor
    {
      public: static const double C;

      /// \brief Constructor
      public: WirelessTransceiver();

      /// \brief Destructor
      public: virtual ~WirelessTransceiver();

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
      public: double GetGain();

      /// \brief Returns the receiver power (dBm).
      /// \return Receiver power (dBm).
      public: double GetPower();

      /// \brief Parent entity
      protected: physics::EntityPtr entity;

      /// \brief Publisher to publish propagation model data
      protected: transport::PublisherPtr pub;

      /// \brief Receiver's power (dBm).
      protected: double power;

      /// \brief Antenna's gain of the receiver (dBi).
      protected: double gain;
    };
    /// \}
  }
}
#endif
