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
#ifndef _BATTERY_PRIVATE_HH_
#define _BATTERY_PRIVATE_HH_

#include <map>
#include <string>

#include "gazebo/physics/PhysicsTypes.hh"

namespace gazebo
{
  namespace physics
  {
    /// \brief Private data class for Battery.
    class BatteryPrivate
    {
      /// \brief Link that contains this battery.
      public: physics::LinkPtr link;

      /// \brief Event connection.
      public: event::ConnectionPtr connection;

      /// \brief Initial voltage in volts.
      public: double initVoltage;

      /// \brief Real voltage in volts.
      public: double curVoltage;

      /// \brief Map of unique consumer ID to power loads in watts.
      public: std::map<uint32_t, double> powerLoads;

      /// \brief The function used to to update the real voltage.
      /// It takes as inputs current voltage and list of power loads.
      public: boost::function<
        double (double, const std::map<uint32_t, double> &)> updateFunc;

      /// \brief Name of the battery.
      public: std::string name;
    };
  }
}

#endif
