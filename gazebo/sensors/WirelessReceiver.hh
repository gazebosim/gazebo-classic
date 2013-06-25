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

#include <vector>
#include <string>

#include "gazebo/physics/PhysicsTypes.hh"

#include "gazebo/transport/TransportTypes.hh"

#include "gazebo/math/Pose.hh"

#include "gazebo/sensors/Sensor.hh"

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
      /// \brief Constructor
      public: WirelessReceiver();

      /// \brief Destructor
      public: virtual ~WirelessReceiver();

      // Documentation inherited
      public: virtual void Load(const std::string &_worldName,
                                sdf::ElementPtr _sdf);

      // Documentation inherited
      public: virtual void Load(const std::string &_worldName);

      // Documentation inherited
      public: virtual void Init();

      protected: virtual void UpdateImpl(bool _force);

      // Documentation inherited
      public: virtual void Fini();

      /// \brief Parent entity
      private: physics::EntityPtr entity;
    };
    /// \}
  }
}
#endif
