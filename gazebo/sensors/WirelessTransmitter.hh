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
#ifndef _GAZEBO_WIRELESS_TRANSMITTER_HH_
#define _GAZEBO_WIRELESS_TRANSMITTER_HH_

#include <string>
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/WirelessTransceiver.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace sensors
  {
    // Forward declare private data class.
    class WirelessTransmitterPrivate;

    /// \addtogroup gazebo_sensors
    /// \{

    /// \class WirelessTransmitter WirelessTransmitter.hh sensors/sensors.hh
    /// \brief Transmitter to send wireless signals
    class GAZEBO_VISIBLE WirelessTransmitter: public WirelessTransceiver
    {
      /// \brief Constructor.
      public: WirelessTransmitter();

      /// \brief Destructor
      public: virtual ~WirelessTransmitter();

      // Documentation inherited
      protected: virtual bool UpdateImpl(const bool _force);

      // Documentation inherited
      public: virtual void Load(const std::string &_worldName);

      // Documentation inherited
      public: virtual void Init();

      /// \brief Returns the Service Set Identifier (network name).
      /// \return Service Set Identifier (network name).
      /// \deprecated See ESSID()
      public: std::string GetESSID() const GAZEBO_DEPRECATED(7.0);

      /// \brief Returns the Service Set Identifier (network name).
      /// \return Service Set Identifier (network name).
      public: std::string ESSID() const;

      /// \brief Returns reception frequency (MHz).
      /// \return Reception frequency (MHz).
      /// \deprecated See Freq()
      public: double GetFreq() const GAZEBO_DEPRECATED(7.0);

      /// \brief Returns reception frequency (MHz).
      /// \return Reception frequency (MHz).
      public: double Freq() const;

      /// \brief Returns the signal strength in a given world's point (dBm).
      /// \param[in] _receiver Pose of the receiver
      /// \param[in] _rxGain Receiver gain value
      /// \return Signal strength in a world's point (dBm).
      public: double SignalStrength(const ignition::math::Pose3d &_receiver,
          const double _rxGain);

      /// \internal
      /// \brief Private data pointer
      private: std::shared_ptr<WirelessTransmitterPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
