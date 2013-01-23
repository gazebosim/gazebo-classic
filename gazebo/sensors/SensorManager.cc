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
#include "sensors/Sensor.hh"
#include "sensors/SensorFactory.hh"
#include "sensors/SensorManager.hh"

using namespace gazebo;
using namespace sensors;

//////////////////////////////////////////////////
SensorManager::SensorManager()
  : stop(true), initialized(false), runThread(NULL)
{
  this->sensorLists.push_back(&this->sensors);
  this->sensorLists.push_back(&this->imageSensors);
}

//////////////////////////////////////////////////
SensorManager::~SensorManager()
{
  this->sensors.clear();
  this->imageSensors.clear();
  this->initSensors.clear();
}

//////////////////////////////////////////////////
void SensorManager::Run()
{
  this->stop = false;
  // this->runThread = new boost::thread(
  //    boost::bind(&SensorManager::RunLoop, this));

  printf("RUN baby\n");
  this->rayThread = new boost::thread(
      boost::bind(&SensorManager::RunRay, this));
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

  if (this->rayThread)
  {
    this->rayThread->join();
    delete this->rayThread;
    this->rayThread = NULL;
  }
}

//////////////////////////////////////////////////
void SensorManager::RunLoop()
{
  while (!this->stop)
    this->Update();
}

//////////////////////////////////////////////////
void SensorManager::RunRay()
{
  Sensor_V::iterator iter;
  Sensor_V::iterator end;

  while (!this->stop)
  {
    {
      //boost::recursive_mutex::scoped_lock lock(this->mutex);
      end = this->sensors.end();
      for (iter = this->sensors.begin(); iter != end; ++iter)
      {
        (*iter)->Update(false);
      }
    }
  }
}

//////////////////////////////////////////////////
void SensorManager::Update(bool _force)
{
  Sensor_V::iterator iter;
  Sensor_V::iterator end;

  {
    boost::recursive_mutex::scoped_lock lock(this->mutex);
    // in case things are spawn, sensors length changes
    end = this->initSensors.end();
    for (iter = this->initSensors.begin(); iter != end; ++iter)
    {
      (*iter)->Init();
      if ((*iter)->GetType() != "camera" &&
          (*iter)->GetType() != "multicamera" &&
          (*iter)->GetType() != "depth")
      {
        this->sensors.push_back(*iter);
      }
      else
        this->imageSensors.push_back(*iter);
    }
    this->initSensors.clear();
  }

  event::Events::preRender();

  // Tell all the cameras to render
  event::Events::render();

  event::Events::postRender();

  {
    //boost::recursive_mutex::scoped_lock lock(this->mutex);
    // in case things are spawn, sensors length changes
    end = this->imageSensors.end();
    for (iter = this->imageSensors.begin(); iter != end; ++iter)
    {
      (*iter)->Update(_force);
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
  Sensor_V::iterator iter;

  for (std::vector<Sensor_V*>::iterator viter = this->sensorLists.begin();
      viter != this->sensorLists.end(); ++viter)
  {
    for (iter = (*viter)->begin(); iter != (*viter)->end(); ++iter)
    {
      (*iter)->Init();
    }
  }

  this->initialized = true;
}

//////////////////////////////////////////////////
void SensorManager::Fini()
{
  boost::recursive_mutex::scoped_lock lock(this->mutex);

  this->initialized = false;
  Sensor_V::iterator iter;

  for (std::vector<Sensor_V*>::iterator viter = this->sensorLists.begin();
      viter != this->sensorLists.end(); ++viter)
  {
    for (iter = (*viter)->begin(); iter != (*viter)->end(); ++iter)
    {
      (*iter)->Fini();
    }

    (*viter)->clear();
  }
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
    if (sensor->GetType() != "camera" && sensor->GetType() != "multicamera" &&
        sensor->GetType() != "depth")
      this->sensors.push_back(sensor);
    else
      this->imageSensors.push_back(sensor);
  }
  else
  {
    boost::recursive_mutex::scoped_lock lock(this->mutex);
    this->initSensors.push_back(sensor);
  }

  return sensor->GetScopedName();
}

//////////////////////////////////////////////////
SensorPtr SensorManager::GetSensor(const std::string &_name)
{
  boost::recursive_mutex::scoped_lock lock(this->mutex);

  SensorPtr result;
  Sensor_V::iterator iter;

  for (iter = this->sensors.begin();
       iter != this->sensors.end() && !result; ++iter)
  {
    if ((*iter)->GetScopedName() == _name)
      result = (*iter);
  }

  for (iter = this->imageSensors.begin();
       iter != this->imageSensors.end() && !result; ++iter)
  {
    if ((*iter)->GetScopedName() == _name)
      result = (*iter);
  }

  // If the sensor was not found, then try to find based on an unscoped
  // name.
  // If multiple sensors exist with the same name, then an error occurs
  // because we don't know which sensor is correct.
  if (!result)
  {
    for (std::vector<Sensor_V*>::iterator viter = this->sensorLists.begin();
         viter != this->sensorLists.end(); ++viter)
    {
      for (iter = (*viter)->begin(); iter != (*viter)->end(); ++iter)
      {
        if ((*iter)->GetName() != _name)
          continue;

        if (!result)
          result = (*iter);
        else
        {
          gzerr << "Unable to get a sensor, multiple sensors with the same "
            << "name[" << _name << "]. Use a scoped name instead, "
            << "world_name::model_name::link_name::sensor_name.\n";
          result.reset();
        }
      }
    }
  }

  return result;
}

//////////////////////////////////////////////////
Sensor_V SensorManager::GetSensors() const
{
  Sensor_V result;

  std::copy(this->sensors.begin(), this->sensors.end(),
            std::back_inserter(result));

  std::copy(this->imageSensors.begin(), this->imageSensors.end(),
            std::back_inserter(result));

  return result;
}

//////////////////////////////////////////////////
void SensorManager::RemoveSensor(const std::string &_name)
{
  SensorPtr sensor = this->GetSensor(_name);

  if (!sensor)
  {
    gzerr << "Unable to remove sensor[" << _name << "]\n";
  }
  else
  {
    boost::recursive_mutex::scoped_lock lock(this->mutex);

    Sensor_V::iterator iter;

    bool removed = false;
    for (std::vector<Sensor_V*>::iterator viter = this->sensorLists.begin();
        viter != this->sensorLists.end() && !removed; ++viter)
    {
      for (iter = (*viter)->begin(); iter != (*viter)->end(); ++iter)
      {
        if ((*iter)->GetScopedName() == sensor->GetScopedName())
        {
          (*iter)->Fini();
          (*viter)->erase(iter);
          removed = true;
        }
      }
    }

    if (!removed)
      gzerr << "RemoveSensor failed. The SensorManager's list of sensors "
        << "changed during sensor removal. This is bad, and should "
        << "never happen.\n";
  }
}

//////////////////////////////////////////////////
void SensorManager::RemoveSensors()
{
  boost::recursive_mutex::scoped_lock lock(this->mutex);
  Sensor_V::iterator iter;
  for (iter = this->sensors.begin(); iter != this->sensors.end(); ++iter)
    (*iter)->Fini();

  for (iter = this->imageSensors.begin();
       iter != this->imageSensors.end(); ++iter)
  {
    (*iter)->Fini();
  }

  this->sensors.clear();
  this->imageSensors.clear();
  this->initSensors.clear();
}
