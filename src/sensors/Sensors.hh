/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#ifndef GAZEBO_SENSORS_HH
#define GAZEBO_SENSORS_HH

#include "SensorTypes.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \addtogroup gazebo_sensors
    /// \{
    /// \brief Load the sensor library
    bool load();

    std::string create_sensor(sdf::ElementPtr _elem,
                              const std::string &_parentName);

    /// \brief Run the sensor generation one step.
    /// \param force: If true, all sensors are forced to update. Otherwise
    ///        a sensor will update based on it's Hz rate.
    void run_once(bool force = true);

    /// \brief Run sensor generation continuously. This is a blocking call
    void run();

    /// \brief Stop the sensor generation loop.
    void stop();

    bool init();
    bool fini();

    // \brief Remove all sensors.
    bool remove_sensors();

    /// \brief Get a sensor by name
    SensorPtr get_sensor(const std::string &_name);

    /// \}
  }
}
#endif

