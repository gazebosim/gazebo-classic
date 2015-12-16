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
#ifndef _GAZEBO_SENSORS_SENSORMANAGER_HH_
#define _GAZEBO_SENSORS_SENSORMANAGER_HH_

#include <string>
#include <vector>
#include <list>
#include <mutex>

#include <sdf/sdf.hh>

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/common/SingletonT.hh"
#include "gazebo/common/UpdateInfo.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  /// \ingroup gazebo_sensors
  /// \brief Sensors namespace
  namespace sensors
  {
    /// \addtogroup gazebo_sensors
    /// \{
    /// \class SensorManager SensorManager.hh sensors/sensors.hh
    /// \brief Class to manage and update all sensors
    class GAZEBO_VISIBLE SensorManager : public SingletonT<SensorManager>
    {
      /// \brief This is a singletone class. Use SensorManager::Instance()
      /// to get a pointer to this class.
      private: SensorManager();

      /// \brief Destructor
      private: virtual ~SensorManager();

      /// \brief Update all the sensors
      ///
      /// Checks to see if any sensor need to be initialized first,
      /// then updates all sensors once.
      /// \param[in] _force True force update, false if not
      public: void Update(const bool _force = false);

      /// \brief Init all the sensors
      public: void Init();

      /// \brief Run sensor updates in separate threads.
      /// This will only run non-image based sensor updates.
      public: void RunThreads();

      /// \brief Stop the run thread
      public: void Stop();

      /// \brief Finalize all the sensors
      public: void Fini();

      /// \brief Get all the sensor types
      /// \param[out] All the sensor types.
      /// \deprecated See SensorTypes(std::vector<std::string> &) const;
      public: void GetSensorTypes(std::vector<std::string> &_types) const
              GAZEBO_DEPRECATED(7.0);

      /// \brief Add a sensor from an SDF element. This function will also Load
      /// and Init the sensor.
      /// \param[in] _elem The SDF element that describes the sensor
      /// \param[in] _worldName Name of the world in which to create the sensor
      /// \param[in] _parentName The name of the parent link which the sensor is
      /// attached to.
      /// \return The name of the sensor
      public: std::string CreateSensor(sdf::ElementPtr _elem,
                                       const std::string &_worldName,
                                       const std::string &_parentName,
                                       const uint32_t _parentId);

      /// \brief Get a sensor
      /// \param[in] _name The name of a sensor to find.
      /// \return A pointer to the sensor. NULL if not found.
      /// \deprecated See Sensor(const std::string &) const
      public: SensorPtr GetSensor(const std::string &_name) const
              GAZEBO_DEPRECATED(7.0);

      /// \brief Get a sensor
      /// \param[in] _name The name of a sensor to find.
      /// \return A pointer to the sensor. NULL if not found.
      public: SensorPtr Sensor(const std::string &_name) const;

      /// \brief Get all the sensors.
      /// \return Vector of all the sensors.
      /// \deprecated See Sensors() const
      public: Sensor_V GetSensors() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get all the sensors.
      /// \return Vector of all the sensors.
      public: Sensor_V Sensors() const;

      /// \brief Remove a sensor
      /// \param[in] _name The name of the sensor to remove.
      public: void RemoveSensor(const std::string &_name);

      /// \brief Remove all sensors
      public: void RemoveSensors();

      /// \brief True if SensorManager::initSensors queue is empty
      /// i.e. all sensors managed by SensorManager have been initialized
      public: bool SensorsInitialized();

      /// \brief Reset last update times in all sensors.
      public: void ResetLastUpdateTimes();

      /// \brief Add a new sensor to a sensor container.
      /// \param[in] _sensor Pointer to a sensor to add.
      private: void AddSensor(SensorPtr _sensor);

      /// \internal
      /// \brief Private data pointer
      private: std::unique_ptr<SensorManagerPrivate> dataPtr;

      /// \brief This is a singleton class.
      private: friend class SingletonT<SensorManager>;

      /// \brief Allow access to sensorTimeMutex member var.
      private: friend class SensorContainer;
    };
    /// \}
  }
}
#endif
