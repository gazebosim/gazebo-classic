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
#ifndef _GAZEBO_BATTERY_PRIVATE_HH_
#define _GAZEBO_BATTERY_PRIVATE_HH_

#include <string>
#include <map>
#include <functional>
#include <mutex>
#include "gazebo/common/CommonTypes.hh"

namespace gazebo
{
  namespace common
  {
    /// \internal
    /// \brief Private data for the Battery class
    class BatteryPrivate
    {
      /// \brief Initial voltage in volts.
      public: double initVoltage;

      /// \brief Real voltage in volts.
      public: double realVoltage;

      /// \brief Map of unique consumer ID to power loads in watts.
      public: std::map<uint32_t, double> powerLoads;

      /// \brief Counter used to produce unique consumer (powerload) ids.
      public: uint32_t powerLoadCounter;

      /// \brief The function used to to update the real voltage.
      /// It takes as inputs current voltage and list of power loads.
      public: std::function<double (const BatteryPtr)> updateFunc;

      /// \brief Name of the battery.
      public: std::string name;

      /// \brief Mutex that protects the powerLoads map
      public: std::mutex powerLoadsMutex;
    };
  }
}
#endif
