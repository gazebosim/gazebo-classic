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
/*
 * Desc: Factory for creating sensor
 * Author: Andrew Howard
 * Date: 18 May 2003
 */

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/Sensor.hh"

void RegisterAltimeterSensor();
void RegisterCameraSensor();
void RegisterContactSensor();
void RegisterDepthCameraSensor();
void RegisterMultiCameraSensor();
void RegisterGpsSensor();
void RegisterGpuRaySensor();
void RegisterImuSensor();
void RegisterMagnetometerSensor();
void RegisterRaySensor();
void RegisterRFIDSensor();
void RegisterRFIDTag();
void RegisterSonarSensor();
void RegisterForceTorqueSensor();
void RegisterWirelessTransmitter();
void RegisterWirelessReceiver();

using namespace gazebo;
using namespace sensors;

std::map<std::string, SensorFactoryFn> SensorFactory::sensorMap;

/////////////////////////////////////////////////
void SensorFactory::RegisterAll()
{
  RegisterAltimeterSensor();
  RegisterCameraSensor();
  RegisterContactSensor();
  RegisterDepthCameraSensor();
  RegisterMultiCameraSensor();
  RegisterGpsSensor();
  RegisterGpuRaySensor();
  RegisterImuSensor();
  RegisterMagnetometerSensor();
  RegisterRaySensor();
  RegisterRFIDSensor();
  RegisterRFIDTag();
  RegisterSonarSensor();
  RegisterForceTorqueSensor();
  RegisterWirelessTransmitter();
  RegisterWirelessReceiver();
}

/////////////////////////////////////////////////
void SensorFactory::RegisterSensor(const std::string &classname,
                                   SensorFactoryFn factoryfn)
{
  sensorMap[classname] = factoryfn;
}

/////////////////////////////////////////////////
SensorPtr SensorFactory::NewSensor(const std::string &classname)
{
  SensorPtr sensor;

  if (sensorMap[classname])
  {
    sensor.reset((sensorMap[classname]) ());
  }

  return sensor;
}

/////////////////////////////////////////////////
void SensorFactory::GetSensorTypes(std::vector<std::string> &_types)
{
  _types.clear();

  std::map<std::string, SensorFactoryFn>::const_iterator iter;
  for (iter = sensorMap.begin(); iter != sensorMap.end(); ++iter)
  {
    _types.push_back(iter->first);
  }
}
