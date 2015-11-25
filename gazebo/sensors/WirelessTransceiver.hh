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
    // Forward declare private data class
    class WirelessTransceiverPrivate;

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
      public: virtual std::string Topic() const;

      // Documentation inherited
      public: virtual void Load(const std::string &_worldName);

      // Documentation inherited
      public: virtual void Init();

      // Documentation inherited
      public: virtual void Fini();

      /// \brief Returns the antenna's gain of the receiver (dBi).
      /// \return Antenna's gain of the receiver (dBi).
      /// \deprecated See Gain()
      public: double GetGain() const GAZEBO_DEPRECATED(7.0);

      /// \brief Returns the antenna's gain of the receiver (dBi).
      /// \return Antenna's gain of the receiver (dBi).
      public: double Gain() const;

      /// \brief Returns the receiver power (dBm).
      /// \return Receiver power (dBm).
      /// \deprecated See Power()
      public: double GetPower() const GAZEBO_DEPRECATED(7.0);

      /// \brief Returns the receiver power (dBm).
      /// \return Receiver power (dBm).
      public: double Power() const;

      /// \internal
      /// \brief Constructor used by inherited classes
      /// \param[in] _dataPtr Pointer to private data.
      protected: WirelessTransceiver(WirelessTransceiverPrivate &_dataPtr);

      /// \internal
      /// \brief Private data pointer
      private: std::shared_ptr<WirelessTransceiverPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
