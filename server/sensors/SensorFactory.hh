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
/*
 * Desc: Factory for creating sensors
 * Author: Andrew Howard
 * Date: 18 May 2003
 * SVN info: $Id$
 */

#ifndef SENSORFACTORY_HH
#define SENSORFACTORY_HH

#include <string>
#include <map>
#include "StaticPluginRegister.hh"

namespace gazebo
{

// Forward declarations
class Sensor;
class Body;


// Prototype for sensor factory functions
typedef Sensor* (*SensorFactoryFn) (Body*);

/// \addtogroup gazebo_sensor
/// \brief The sensor factory; the class is just for namespacing purposes.
/// \{

/// \brief The sensor factory; the class is just for namespacing purposes.
class SensorFactory
{
  /// \brief Register all known sensors.
  //public: static void RegisterAll();
  
  /// \brief Register a sensor class (called by sensor registration function).
  public: static void RegisterSensor(std::string type, std::string  classname,
                                    SensorFactoryFn factoryfn);

  /// \brief Create a new instance of a sensor.  Used by the world when
  /// reading the world file.
  public: static Sensor *NewSensor(const std::string &classname, Body *body);

  /// \brief A list of registered sensor classes
  private: static std::map<std::string, SensorFactoryFn> sensors;
};


/// \brief Static sensor registration macro
///
/// Use this macro to register sensors with the server.
/// @param name Sensor type name, as it appears in the world file.
/// @param classname C++ class name for the sensor.
#define GZ_REGISTER_STATIC_SENSOR(name, classname) \
Sensor *New##classname(Body *body) \
{ \
  return new classname(body); \
} \
void Register##classname() \
{\
  SensorFactory::RegisterSensor("static", name, New##classname);\
}\
StaticPluginRegister Registered##classname (Register##classname);

/// \}
}

#endif
