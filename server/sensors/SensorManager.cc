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
 * Desc: Class to manager all sensors
 * Author: Nate Koenig
 * Date: 18 Dec 2009
 * SVN info: $Id$
 */

#include "Sensor.hh"
#include "SensorManager.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
SensorManager::SensorManager()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
SensorManager::~SensorManager()
{
  this->sensors.erase(this->sensors.begin(), this->sensors.end());
}

////////////////////////////////////////////////////////////////////////////////
/// Update all the sensors
void SensorManager::Update()
{
  std::list<Sensor*>::iterator iter;
  for (iter = this->sensors.begin(); iter != this->sensors.end(); iter++)
    (*iter)->Update();
}

////////////////////////////////////////////////////////////////////////////////
/// Init all the sensors
void SensorManager::Init()
{
  std::list<Sensor*>::iterator iter;
  for (iter = this->sensors.begin(); iter != this->sensors.end(); iter++)
    (*iter)->Init();
}

////////////////////////////////////////////////////////////////////////////////
/// Finalize all the sensors
void SensorManager::Fini()
{
  std::list<Sensor*>::iterator iter;
  for (iter = this->sensors.begin(); iter != this->sensors.end(); iter++)
    (*iter)->Fini();
}


////////////////////////////////////////////////////////////////////////////////
/// Add a sensor
void SensorManager::AddSensor(Sensor *sensor)
{
  if (!sensor)
    return;

  this->sensors.push_back(sensor);
}

////////////////////////////////////////////////////////////////////////////////
/// Remove a sensor
void SensorManager::RemoveSensor(Sensor *sensor)
{
  if (!sensor)
    return;

  std::list<Sensor*>::iterator iter;
  for (iter = this->sensors.begin(); iter != this->sensors.end(); iter++)
    if (*iter == sensor)
      break;

  if (iter != this->sensors.end())
  {
    delete (*iter);
    this->sensors.erase(iter);
  }
}
