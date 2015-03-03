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

#include "gazebo/common/Console.hh"

#include "gazebo/transport/TransportIface.hh"

#include "gazebo/rendering/RenderingIface.hh"

#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/sensors/Sensor.hh"
#include "gazebo/sensors/SensorsIface.hh"

using namespace gazebo;

bool g_disable = false;

/////////////////////////////////////////////////
bool sensors::load()
{
  if (g_disable)
    return true;

  // Register all the sensor types
  sensors::SensorFactory::RegisterAll();

  // Load the rendering system
  return gazebo::rendering::load();
}

/////////////////////////////////////////////////
bool sensors::init()
{
  if (g_disable)
    return true;

  // The rendering engine will run headless
  if (!gazebo::rendering::init())
  {
    gzthrow("Unable to intialize the rendering engine");
    return false;
  }

  sensors::SensorManager::Instance()->Init();

  return true;
}

/////////////////////////////////////////////////
bool sensors::fini()
{
  if (g_disable)
    return true;

  sensors::SensorManager::Instance()->Fini();
  rendering::fini();
  return true;
}

/////////////////////////////////////////////////
std::string sensors::create_sensor(sdf::ElementPtr _elem,
                                   const std::string &_worldName,
                                   const std::string &_parentName,
                                   uint32_t _parentId)
{
  if (g_disable)
    return "";

  return sensors::SensorManager::Instance()->CreateSensor(_elem, _worldName,
      _parentName, _parentId);
}

/////////////////////////////////////////////////
void sensors::remove_sensor(const std::string &_sensorName)
{
  if (g_disable)
    return;

  sensors::SensorManager::Instance()->RemoveSensor(_sensorName);
}

/////////////////////////////////////////////////
void sensors::run_threads()
{
  if (g_disable)
    return;

  sensors::SensorManager::Instance()->RunThreads();
}

/////////////////////////////////////////////////
void sensors::run_once(bool _force)
{
  if (g_disable)
    return;

  sensors::SensorManager::Instance()->Update(_force);
}

/////////////////////////////////////////////////
void sensors::stop()
{
  if (g_disable)
    return;

  sensors::SensorManager::Instance()->Stop();
}

/////////////////////////////////////////////////
bool sensors::remove_sensors()
{
  if (g_disable)
    return true;

  sensors::SensorManager::Instance()->RemoveSensors();
  return true;
}

/////////////////////////////////////////////////
sensors::SensorPtr sensors::get_sensor(const std::string &_name)
{
  if (g_disable)
    return sensors::SensorPtr();

  return sensors::SensorManager::Instance()->GetSensor(_name);
}

/////////////////////////////////////////////////
void sensors::disable()
{
  g_disable = true;
}

/////////////////////////////////////////////////
void sensors::enable()
{
  g_disable = false;
}
