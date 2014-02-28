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
#ifndef _GAZEBO_SENSORSIFACE_HH_
#define _GAZEBO_SENSORSIFACE_HH_

#include <string>
#include <sdf/sdf.hh>
#include "gazebo/sensors/SensorTypes.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \addtogroup gazebo_sensors
    /// \{

    /// \brief Load the sensor library.
    /// \return True if successfully loaded, false if not.
    bool load();

    /// \brief Deprecated.
    std::string create_sensor(sdf::ElementPtr _elem,
        const std::string &_worldName,
        const std::string &_parentName) GAZEBO_DEPRECATED(2.0);

    /// \brief Create a sensor using SDF.
    /// \param[in] _elem The SDF element that describes the sensor.
    /// \param[in] _worldName Name of the world in which to create the sensor.
    /// \param[in] _parentName The fully scoped parent name (model::link).
    /// \return The name of the new sensor.
    std::string create_sensor(sdf::ElementPtr _elem,
        const std::string &_worldName, const std::string &_parentName,
        uint32_t _parentId);

    /// \brief Remove a sensor by name
    /// \param[in] _sensorName Name of sensor to remove
    void remove_sensor(const std::string &_sensorName);

    /// \brief Run the sensor generation one step.
    /// \param _force: If true, all sensors are forced to update. Otherwise
    ///        a sensor will update based on it's Hz rate.
    void run_once(bool _force = false);

    /// \brief Run sensors in a threads. This is a non-blocking call.
    void run_threads();

    /// \brief Stop the sensor generation loop.
    void stop();

    /// \brief initialize the sensor generation loop.
    /// \return True if successfully initialized, false if not
    bool init();

    /// \brief shutdown the sensor generation loop.
    /// \return True if successfully finalized, false if not
    bool fini();

    /// \brief Remove all sensors.
    /// \return True if all successfully removed, false if not
    bool remove_sensors();

    /// \brief Get a sensor using by name
    ///
    /// The given name should have:
    /// world_name::model_name::link_name::sensor_name
    /// \param[in] _name Name of the sensor. This name should be fully scoped.
    /// This means _name = world_name::model_name::link_name::sensor_name.
    /// You may use the unscoped sensor name if that name is unique within
    /// the entire simulation. If the name is not unique a NULL pointer is
    /// returned.
    /// \return Pointer to the sensor, NULL if the sensor could not be
    /// found.
    SensorPtr get_sensor(const std::string &_name);

    /// \brief Disable sensors.
    void disable();

    /// \brief Enable sensors.
    void enable();
    /// \}
  }
}
#endif
