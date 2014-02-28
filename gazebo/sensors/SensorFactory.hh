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
/*
 * Desc: Factory for creating sensors
 * Author: Andrew Howard
 * Date: 18 May 2003
 */

#ifndef _SENSORFACTORY_HH_
#define _SENSORFACTORY_HH_

#include <string>
#include <map>
#include <vector>

#include "gazebo/sensors/SensorTypes.hh"

namespace gazebo
{
  /// \ingroup gazebo_sensors
  /// \brief Sensors namespace
  namespace sensors
  {
  /// \def Sensor
  /// \brief Prototype for sensor factory functions
  typedef Sensor* (*SensorFactoryFn) ();

  /// \addtogroup gazebo_sensors
  /// \{
  /// \class SensorFactor SensorFactory.hh sensors/sensors.hh
  /// \brief The sensor factory; the class is just for namespacing purposes.
  class SensorFactory
  {
    /// \brief Register all known sensors
    ///  \li sensors::CameraSensor
    ///  \li sensors::DepthCameraSensor
    ///  \li sensors::GpuRaySensor
    ///  \li sensors::RaySensor
    ///  \li sensors::ContactSensor
    ///  \li sensors::RFIDSensor
    ///  \li sensors::RFIDTag
    ///  \li sensors::WirelessTransmitter
    ///  \li sensors::WirelessReceiver
    public: static void RegisterAll();

    /// \brief Register a sensor class (called by sensor registration function).
    /// \param[in] _className Name of class of sensor to register.
    /// \param[in] _factoryfn Function handle for registration.
    public: static void RegisterSensor(const std::string &_className,
                                       SensorFactoryFn _factoryfn);

    /// \brief Create a new instance of a sensor.  Used by the world when
    /// reading the world file.
    /// \param[in] _className Name of sensor class
    /// \return Pointer to Sensor
    public: static SensorPtr NewSensor(const std::string &_className);

    /// \brief Get all the sensor types
    /// \param _types Vector of strings of the sensor types,
    /// populated by function
    public: static void GetSensorTypes(std::vector<std::string> &_types);

    /// \brief A list of registered sensor classes
    private: static std::map<std::string, SensorFactoryFn> sensorMap;
  };


  /// \brief Static sensor registration macro
  ///
  /// Use this macro to register sensors with the server.
  /// @param name Sensor type name, as it appears in the world file.
  /// @param classname C++ class name for the sensor.
  #define GZ_REGISTER_STATIC_SENSOR(name, classname) \
  Sensor *New##classname() \
  { \
    return new gazebo::sensors::classname(); \
  } \
  void Register##classname() \
  {\
    SensorFactory::RegisterSensor(name, New##classname);\
  }
  /// \}
  }
}

#endif


