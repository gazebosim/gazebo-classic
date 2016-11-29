/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_SENSORS_WIRELESSRECEIVER_HH_
#define _GAZEBO_SENSORS_WIRELESSRECEIVER_HH_

#include <memory>
#include <string>
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/sensors/WirelessTransceiver.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace sensors
  {
    // Forward declare private data
    class WirelessReceiverPrivate;

    /// \addtogroup gazebo_sensors
    /// \{

    /// \class WirelessReceiver WirelessReceiver.hh sensors/sensors.hh
    /// \brief Sensor class for receiving wireless signals.
    class GAZEBO_VISIBLE WirelessReceiver: public WirelessTransceiver
    {
      /// \brief Constructor
      public: WirelessReceiver();

      /// \brief Destructor
      public: virtual ~WirelessReceiver();

      // Documentation inherited
      public: virtual void Load(const std::string &_worldName);

      // Documentation inherited
      public: virtual void Init();

      // Documentation inherited
      public: virtual void Fini();

      /// \brief Returns the minimum frequency filtered (MHz).
      /// \return Reception frequency (MHz).
      public: double MinFreqFiltered() const;

      /// \brief Returns the maximum frequency filtered (MHz).
      /// \return Reception frequency (MHz).
      public: double MaxFreqFiltered() const;

      /// \brief Returns the receiver sensitivity (dBm).
      /// \return Receiver sensitivity (dBm).
      public: double Sensitivity() const;

      // Documentation inherited
      protected: virtual bool UpdateImpl(const bool _force);

      /// \internal
      /// \brief Private data pointer
      private: std::unique_ptr<WirelessReceiverPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
