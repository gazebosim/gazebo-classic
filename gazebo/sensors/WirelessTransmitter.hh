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
/* Desc: Wireless transmitter
 * Author: Carlos Agüero
 * Date: 24 June 2013
 */

#ifndef _WIRELESS_TRANSMITTER_HH_
#define _WIRELESS_TRANSMITTER_HH_

#include <vector>
#include <string>

#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/sensors/Sensor.hh"
#include "gazebo/math/gzmath.hh"
#include "gazebo/physics/physics.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \addtogroup gazebo_sensors
    /// \{

    /// \class WirelessTransmitter WirelessTransmitter.hh sensors/sensors.hh
    /// \brief Transmitter to send wireless signals 
    class WirelessTransmitter: public Sensor
    {
      /// \brief Constructor.
      public: WirelessTransmitter();

      /// \brief Destructor.
      public: virtual ~WirelessTransmitter();

      // Documentation inherited
      public: virtual void Load(const std::string & _worldName,
                                sdf::ElementPtr &_sdf);

      // Documentation inherited
      public: virtual void Load(const std::string & _worldName);

      // Documentation inherited
      public: virtual void Init();

      // Documentation inherited
      protected: virtual void UpdateImpl(bool _force);

      // Documentation inherited
      public: virtual void Fini();

      /// \brief Returns pose of transmitter in world coordinate.
      /// \return Pose of object.
      public: math::Pose GetTransmitterPose() const
              {return entity->GetWorldPose();}

      /// \brief Pointer the entity that has the transmitter.
      private: physics::EntityPtr entity;
    };
    /// \}
  }
}
#endif
