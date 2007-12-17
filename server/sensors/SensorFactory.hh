/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
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
  public: static void RegisterAll();
  
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
}

/// \}
}

#endif
