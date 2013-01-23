/*
 * Copyright 2012 Open Source Robotics Foundation
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
/*
 * Desc: Class to manager all sensors
 * Author: Nate Koenig
 * Date: 18 Dec 2009
 */

#ifndef _SENSORMANAGER_HH_
#define _SENSORMANAGER_HH_

#include <boost/thread.hpp>
#include <string>
#include <vector>

#include "common/SingletonT.hh"
#include "sensors/SensorTypes.hh"
#include "sdf/sdf.hh"

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
    class SensorManager : public SingletonT<SensorManager>
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
      public: void Update(bool _force = false);

      /// \brief Init all the sensors
      public: void Init();

      /// \brief Run the sensor manager update in a new thread
      public: void Run();

      /// \brief Stop the run thread
      public: void Stop();

      /// \brief Finalize all the sensors
      public: void Fini();

      /// \brief Get all the sensor types
      /// \param[out] All the sensor types.
      public: void GetSensorTypes(std::vector<std::string> &_types) const;

      /// \brief Add a sensor from an SDF element. This function will also Load
      /// and Init the sensor.
      /// \param[in] _elem The SDF element that describes the sensor
      /// \param[in] _worldName Name of the world in which to create the sensor
      /// \param[in] _parentName The name of the parent link which the sensor is
      /// attached to.
      /// \return The name of the sensor
      public: std::string CreateSensor(sdf::ElementPtr _elem,
                                       const std::string &_worldName,
                                       const std::string &_parentName);

      /// \brief Get a sensor
      /// \param[in] _name The name of a sensor to find.
      /// \return A pointer to the sensor. NULL if not found.
      public: SensorPtr GetSensor(const std::string &_name);

      /// \brief Get all the sensors.
      /// \return Vector of all the sensors.
      public: Sensor_V GetSensors() const;

      /// \brief Remove a sensor
      /// \param[in] _name The name of the sensor to remove.
      public: void RemoveSensor(const std::string &_name);

      /// \brief Remove all sensors
      public: void RemoveSensors();

      /// \brief True if SensorManager::initSensors queue is empty
      /// i.e. all sensors managed by SensorManager have been initialized
      public: bool SensorsInitialized();

      /// \brief Update loop
      private: void RunLoop();

      private: void RunRay();

      /// \brief If True the sensor manager stop processing sensors.
      private: bool stop;

      /// \brief True if SensorManager::Init has been called
      ///        i.e. SensorManager::sensors are initialized.
      private: bool initialized;

      /// \brief The thread to run sensor updates in.
      private: boost::thread *runThread;

      /// \brief Mutex used when adding and removing sensors.
      private: boost::recursive_mutex mutex;

      /// \brief The list of initialized non-image sensors.
      private: Sensor_V sensors;

      /// \brief The list of initialized image sensors.
      private: Sensor_V imageSensors;

      /// \brief List of sensors that require initialization.
      private: Sensor_V initSensors;

      private: std::vector<Sensor_V*> sensorLists;

      /// \brief Thread for ray sensors.
      private: boost::thread *rayThread;

      /// \brief This is a singleton class.
      private: friend class SingletonT<SensorManager>;
    };
    /// \}
  }
}
#endif


