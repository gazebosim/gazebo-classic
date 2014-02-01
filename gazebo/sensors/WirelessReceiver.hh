/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#ifndef _WIRELESS_RECEIVER_HH_
#define _WIRELESS_RECEIVER_HH_

#include <string>
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/sensors/WirelessTransceiver.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \addtogroup gazebo_sensors
    /// \{

    /// \class WirelessReceiver WirelessReceiver.hh sensors/sensors.hh
    /// \brief Sensor class for receiving wireless signals.
    class WirelessReceiver: public WirelessTransceiver
    {
      /// \brief Constructor
      public: WirelessReceiver();

      /// \brief Constructor
      public: virtual ~WirelessReceiver();

      // Documentation inherited
      public: virtual void Load(const std::string &_worldName);

      // Documentation inherited
      public: virtual void Init();

      // Documentation inherited
      public: virtual void Fini();

      // Documentation inherited
      private: virtual void UpdateImpl(bool _force);

      /// \brief Returns the minimum frequency filtered (MHz).
      /// \return Reception frequency (MHz).
      public: double GetMinFreqFiltered() const;

      /// \brief Returns the maximum frequency filtered (MHz).
      /// \return Reception frequency (MHz).
      public: double GetMaxFreqFiltered() const;

      /// \brief Returns the receiver sensitivity (dBm).
      /// \return Receiver sensitivity (dBm).
      public: double GetSensitivity() const;

      /// \brief Reception low filter frequency (MHz).
      private: double minFreq;

      /// \brief Reception high filter frequency (MHz).
      private: double maxFreq;

      /// \brief Antenna's sensitivity of the receiver (dBm).
      private: double sensitivity;
    };
    /// \}
  }
}
#endif
