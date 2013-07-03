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
/* Desc: Wireless receiver
 * Author: Carlos Agüero
 * Date: 24 June 2013
 */

#ifndef _WIRELESS_RECEIVER_HH_
#define _WIRELESS_RECEIVER_HH_

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/sensors/Sensor.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  /// \ingroup gazebo_sensors
  /// \brief Sensors namespace
  namespace sensors
  {
    class WirelessReceiver;

    /// \addtogroup gazebo_sensors
    /// \{

    /// \class WirelessReceiver WirelessReceiver.hh sensors/sensors.hh
    /// \brief Sensor class for receiving wireless signals.
    class WirelessReceiver: public Sensor
    {
      public: static const double N;
      public: static const double C;

      /// \brief Constructor
      public: WirelessReceiver();

      /// \brief Destructor
      public: virtual ~WirelessReceiver();

      // Documentation inherited                                                
      public: virtual std::string GetTopic() const;

      // Documentation inherited
      public: virtual void Load(const std::string &_worldName);

      // Documentation inherited
      public: virtual void Init();

      // Documentation inherited
      protected: virtual void UpdateImpl(bool _force);

      // Documentation inherited
      public: virtual void Fini();

      /// \brief Returns reception frequency (MHz).
      /// \return Reception frequency (MHz).
      public: double GetFreq();

      /// \brief Returns the antenna's gain of the receiver (dBi).
      /// \return Antenna's gain of the receiver (dBi).
      public: double GetGain();

      /// \brief Returns the receiver power (dBm).
      /// \return Receiver power (dBm).
      public: double GetPower();

      /// \brief Parent entity
      private: physics::EntityPtr entity;

      /// \brief Publisher to publish propagation model data
      private: transport::PublisherPtr pub;

      /// \brief Reception frequency (MHz).
      private: double freq;

      /// \brief Receiver's power (dBm).
      private: double power;

      /// \brief Antenna's gain of the receiver (dBi).
      private: double gain;
    };
    /// \}
  }
}
#endif
