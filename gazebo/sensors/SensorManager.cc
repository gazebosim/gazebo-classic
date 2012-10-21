/*
 * Copyright 2011 Nate Koenig
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
#include "rendering/RenderEngine.hh"
#include "sensors/Sensor.hh"
#include "sensors/SensorFactory.hh"
#include "sensors/SensorManager.hh"

using namespace gazebo;
using namespace sensors;

//////////////////////////////////////////////////
SensorManager::SensorManager()
  : stop(true), initialized(false), runThread(NULL)
{
}

//////////////////////////////////////////////////
SensorManager::~SensorManager()
{
  this->sensors.clear();
  this->initSensors.clear();
}

//////////////////////////////////////////////////
void SensorManager::Run()
{
  this->stop = false;
  this->runThread = new boost::thread(
      boost::bind(&SensorManager::RunLoop, this));
}

//////////////////////////////////////////////////
void SensorManager::Stop()
{
  this->stop = true;
  if (this->runThread)
  {
    this->runThread->join();
    delete this->runThread;
    this->runThread = NULL;
  }
}

//////////////////////////////////////////////////
void SensorManager::RunLoop()
{
  while (!this->stop)
    this->Update();
}

//////////////////////////////////////////////////
void SensorManager::Update(bool force)
{
  std::list<SensorPtr>::iterator iter;
  std::list<SensorPtr>::iterator end;

  {
    boost::recursive_mutex::scoped_lock lock(this->mutex);
    // in case things are spawn, sensors length changes
    end = this->initSensors.end();
    for (iter = this->initSensors.begin(); iter != end; ++iter)
    {
      (*iter)->Init();
      this->sensors.push_back((*iter));
    }
    this->initSensors.clear();
  }

  event::Events::preRender();

  // Tell all the cameras to render
  event::Events::render();

  event::Events::postRender();

  {
    boost::recursive_mutex::scoped_lock lock(this->mutex);
    // in case things are spawn, sensors length changes
    end = this->sensors.end();
    for (iter = this->sensors.begin(); iter != end; ++iter)
    {
      (*iter)->Update(force);
    }
  }
}

//////////////////////////////////////////////////
bool SensorManager::SensorsInitialized()
{
  boost::recursive_mutex::scoped_lock lock(this->mutex);
  bool result = this->initSensors.empty();
  return result;
}

//////////////////////////////////////////////////
void SensorManager::Init()
{
  boost::recursive_mutex::scoped_lock lock(this->mutex);
  std::list<SensorPtr>::iterator iter;
  for (iter = this->sensors.begin(); iter != this->sensors.end(); ++iter)
    (*iter)->Init();
  this->initialized = true;
}

//////////////////////////////////////////////////
void SensorManager::Fini()
{
  boost::recursive_mutex::scoped_lock lock(this->mutex);

  this->initialized = false;
  std::list<SensorPtr>::iterator iter;
  for (iter = this->sensors.begin(); iter != this->sensors.end(); ++iter)
    (*iter)->Fini();
  this->sensors.clear();
}

//////////////////////////////////////////////////
void SensorManager::GetSensorTypes(std::vector<std::string> &_types) const
{
  sensors::SensorFactory::GetSensorTypes(_types);
}

//////////////////////////////////////////////////
std::string SensorManager::CreateSensor(sdf::ElementPtr _elem,
                                        const std::string &_worldName,
                                        const std::string &_parentName)
{
  std::string type = _elem->GetValueString("type");
  SensorPtr sensor = sensors::SensorFactory::NewSensor(type);

  if (!sensor)
  {
    gzerr << "Unable to create sensor of type[" << type << "]\n";
    return std::string();
  }

  // Must come before sensor->Load
  sensor->SetParent(_parentName);

  sensor->Load(_worldName, _elem);

  if (!this->initialized)
  {
    this->sensors.push_back(sensor);
  }
  else
  {
    boost::recursive_mutex::scoped_lock lock(this->mutex);
    this->initSensors.push_back(sensor);
  }

  return sensor->GetName();
}

//////////////////////////////////////////////////
SensorPtr SensorManager::GetSensor(const std::string &_name)
{
  boost::recursive_mutex::scoped_lock lock(this->mutex);

  SensorPtr result;
  std::list<SensorPtr>::iterator iter;
  for (iter = this->sensors.begin(); iter != this->sensors.end(); ++iter)
  {
    if ((*iter)->GetName() == _name)
      result = (*iter);
  }

  return result;
}

//////////////////////////////////////////////////
void SensorManager::RemoveSensor(const std::string &_name)
{
  boost::recursive_mutex::scoped_lock lock(this->mutex);

  std::list<SensorPtr>::iterator iter;
  for (iter = this->sensors.begin(); iter != this->sensors.end(); ++iter)
    if ((*iter)->GetName() == _name)
      break;

  if (iter != this->sensors.end())
  {
    (*iter)->Fini();
    this->sensors.erase(iter);
  }
}

//////////////////////////////////////////////////
void SensorManager::RemoveSensors()
{
  boost::recursive_mutex::scoped_lock lock(this->mutex);
  std::list<SensorPtr>::iterator iter;
  for (iter = this->sensors.begin(); iter != this->sensors.end(); ++iter)
    (*iter)->Fini();

  this->sensors.clear();
  this->initSensors.clear();
}
