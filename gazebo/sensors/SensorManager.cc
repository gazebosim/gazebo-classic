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
  : stop(true), initialized(false), runThread(NULL),
    mutex(new boost::recursive_mutex())
{
}

//////////////////////////////////////////////////
SensorManager::~SensorManager()
{
  delete this->mutex;
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

  this->mutex->lock();
  // in case things are spawn, sensors length changes
  std::list<SensorPtr>::iterator end = this->initSensors.end();
  for (iter = this->initSensors.begin(); iter != end; ++iter)
  {
    (*iter)->Init();
    this->sensors.push_back((*iter));
  }
  this->initSensors.clear();
  this->mutex->unlock();

  event::Events::preRender();

  // Tell all the cameras to render
  event::Events::render();

  event::Events::postRender();

  this->mutex->lock();
  // in case things are spawn, sensors length changes
  end = this->sensors.end();
  for (iter = this->sensors.begin(); iter != end; ++iter)
  {
    (*iter)->Update(force);
  }
  this->mutex->unlock();
}

//////////////////////////////////////////////////
bool SensorManager::SensorsInitialized()
{
  this->mutex->lock();
  bool result = this->initSensors.empty();
  this->mutex->unlock();
  return result;
}

//////////////////////////////////////////////////
void SensorManager::Init()
{
  this->mutex->lock();
  std::list<SensorPtr>::iterator iter;
  for (iter = this->sensors.begin(); iter != this->sensors.end(); ++iter)
    (*iter)->Init();
  this->mutex->unlock();
  this->initialized = true;
}

//////////////////////////////////////////////////
void SensorManager::Fini()
{
  this->initialized = false;
  this->mutex->lock();
  std::list<SensorPtr>::iterator iter;
  for (iter = this->sensors.begin(); iter != this->sensors.end(); ++iter)
    (*iter)->Fini();
  this->sensors.clear();
  this->mutex->unlock();
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
    this->mutex->lock();
    this->initSensors.push_back(sensor);
    this->mutex->unlock();
  }

  return sensor->GetName();
}

//////////////////////////////////////////////////
SensorPtr SensorManager::GetSensor(const std::string &_name)
{
  SensorPtr result;

  this->mutex->lock();
  std::list<SensorPtr>::iterator iter;
  for (iter = this->sensors.begin(); iter != this->sensors.end(); ++iter)
  {
    if ((*iter)->GetName() == _name)
      result = (*iter);
  }
  this->mutex->unlock();

  return result;
}

//////////////////////////////////////////////////
void SensorManager::RemoveSensor(const std::string &_name)
{
  this->mutex->lock();
  std::list<SensorPtr>::iterator iter;
  for (iter = this->sensors.begin(); iter != this->sensors.end(); ++iter)
    if ((*iter)->GetName() == _name)
      break;

  if (iter != this->sensors.end())
    this->sensors.erase(iter);
  this->mutex->unlock();
}

//////////////////////////////////////////////////
void SensorManager::RemoveSensors()
{
  this->mutex->lock();
  std::list<SensorPtr>::iterator iter;
  for (iter = this->sensors.begin(); iter != this->sensors.end(); ++iter)
    (*iter)->Fini();

  this->sensors.clear();
  this->initSensors.clear();
  this->mutex->unlock();
}


