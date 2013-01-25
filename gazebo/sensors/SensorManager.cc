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
  : initialized(false)
{
  // sensors::IMAGE container
  this->sensorContainers.push_back(new ImageSensorContainer());

  // sensors::RAY container
  this->sensorContainers.push_back(new SensorContainer());

  // sensors::OTHER container
  this->sensorContainers.push_back(new SensorContainer());
}

//////////////////////////////////////////////////
SensorManager::~SensorManager()
{
  // Clean up the sensors.
  for (SensorContainer_V::iterator iter = this->sensorContainers.begin();
       iter != this->sensorContainers.end(); ++iter)
  {
    (*iter)->Stop();
    (*iter)->RemoveSensors();
    delete (*iter);
  }
  this->sensorContainers.clear();

  this->initSensors.clear();
}

//////////////////////////////////////////////////
void SensorManager::Run()
{
  // Start the non-image sensor containers. The first item in the
  // sensorsContainers list are the image-based sensors, which rely on the
  // rendering engine, which in turn requires that they run in the main
  // thread.
  for (SensorContainer_V::iterator iter = ++this->sensorContainers.begin();
       iter != this->sensorContainers.end(); ++iter)
  {
    (*iter)->Run();
  }
}

//////////////////////////////////////////////////
void SensorManager::Stop()
{
  // Start all the sensor containers.
  for (SensorContainer_V::iterator iter = this->sensorContainers.begin();
       iter != this->sensorContainers.end(); ++iter)
  {
    (*iter)->Stop();
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
      this->sensorContainers[(*iter)->GetCategory()]->AddSensor(*iter);
    }
    this->initSensors.clear();
  }

  this->sensorContainers[sensors::IMAGE]->Update(_force);
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

  // Initialize all the sensor containers.
  for (SensorContainer_V::iterator iter = this->sensorContainers.begin();
       iter != this->sensorContainers.end(); ++iter)
  {
    (*iter)->Init();
  }

  this->initialized = true;
}

//////////////////////////////////////////////////
void SensorManager::Fini()
{
  boost::recursive_mutex::scoped_lock lock(this->mutex);

  // Finalize all the sensor containers.
  for (SensorContainer_V::iterator iter = this->sensorContainers.begin();
       iter != this->sensorContainers.end(); ++iter)
  {
    (*iter)->Init();
  }

  this->initialized = false;
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

  // Load the sensor
  sensor->Load(_worldName, _elem);

  // If the SensorManager has not been initialized, then it's okay to push
  // the sensor into one of the sensor vectors because the sensor will get
  // initialized in SensorManager::Init
  if (!this->initialized)
  {
    this->sensorContainers[sensor->GetCategory()]->AddSensor(sensor);
  }
  // Otherwise the SensorManager is already running, and the sensor will get
  // initialized during the next SensorManager::Update call.
  else
  {
    boost::recursive_mutex::scoped_lock lock(this->mutex);
    this->initSensors.push_back(sensor);
  }

  return sensor->GetScopedName();
}

//////////////////////////////////////////////////
SensorPtr SensorManager::GetSensor(const std::string &_name) const
{
  boost::recursive_mutex::scoped_lock lock(this->mutex);

  SensorContainer_V::const_iterator iter;
  SensorPtr result;

  // Try to find the sensor in all of the containers
  for (iter = this->sensorContainers.begin();
       iter != this->sensorContainers.end() && !result; ++iter)
  {
    result = (*iter)->GetSensor(_name);
  }

  // If the sensor was not found, then try to find based on an unscoped
  // name.
  // If multiple sensors exist with the same name, then an error occurs
  // because we don't know which sensor is correct.
  if (!result)
  {
    for (iter = this->sensorContainers.begin();
         iter != this->sensorContainers.end(); ++iter)
    {
      if (!(*iter)->GetSensor(_name, true))
        continue;

      if (!result)
        result = (*iter)->GetSensor(_name, true);
      else
      {
        gzerr << "Unable to get a sensor, multiple sensors with the same "
          << "name[" << _name << "]. Use a scoped name instead, "
          << "world_name::model_name::link_name::sensor_name.\n";
        result.reset();
        break;
      }
    }
  }

  return result;
}

//////////////////////////////////////////////////
Sensor_V SensorManager::GetSensors() const
{
  boost::recursive_mutex::scoped_lock lock(this->mutex);
  Sensor_V result;

  // Copy the sensor pointers
  for (SensorContainer_V::const_iterator iter = this->sensorContainers.begin();
       iter != this->sensorContainers.end(); ++iter)
  {
    std::copy((*iter)->sensors.begin(), (*iter)->sensors.end(),
              std::back_inserter(result));
  }

  return result;
}

//////////////////////////////////////////////////
void SensorManager::RemoveSensor(const std::string &_name)
{
  SensorPtr sensor = this->GetSensor(_name);

  if (!sensor)
  {
    gzerr << "Unable to remove sensor[" << _name << "] because it "
          << "does not exist.\n";
  }
  else
  {
    bool removed = false;

    for (SensorContainer_V::iterator iter = this->sensorContainers.begin();
         iter != this->sensorContainers.end() && !removed; ++iter)
    {
      removed = (*iter)->RemoveSensor(sensor->GetScopedName());
    }

    if (!removed)
    {
      gzerr << "RemoveSensor failed. The SensorManager's list of sensors "
            << "changed during sensor removal. This is bad, and should "
            << "never happen.\n";
    }
  }
}

//////////////////////////////////////////////////
void SensorManager::RemoveSensors()
{
  boost::recursive_mutex::scoped_lock lock(this->mutex);

  for (SensorContainer_V::iterator iter = this->sensorContainers.begin();
      iter != this->sensorContainers.end(); ++iter)
  {
    (*iter)->RemoveSensors();
  }
  this->initSensors.clear();
}


//////////////////////////////////////////////////
SensorManager::SensorContainer::SensorContainer()
{
  this->stop = true;
  this->initialized = false;
  this->runThread = NULL;
}

//////////////////////////////////////////////////
SensorManager::SensorContainer::~SensorContainer()
{
  this->sensors.clear();
}

//////////////////////////////////////////////////
void SensorManager::SensorContainer::Init()
{
  boost::recursive_mutex::scoped_lock lock(this->mutex);

  Sensor_V::iterator iter;
  for (iter = this->sensors.begin(); iter != this->sensors.end(); ++iter)
  {
    (*iter)->Init();
  }

  this->initialized = true;
}

//////////////////////////////////////////////////
void SensorManager::SensorContainer::Fini()
{
  boost::recursive_mutex::scoped_lock lock(this->mutex);

  Sensor_V::iterator iter;

  // Finialize each sensor in the current sensor vector
  for (iter = this->sensors.begin(); iter != this->sensors.end(); ++iter)
  {
    (*iter)->Fini();
  }

  // Remove all the sensors from the current sensor vector.
  this->sensors.clear();

  this->initialized = false;
}

//////////////////////////////////////////////////
void SensorManager::SensorContainer::Run()
{
  this->runThread = new boost::thread(
      boost::bind(&SensorManager::SensorContainer::RunLoop, this));
}

//////////////////////////////////////////////////
void SensorManager::SensorContainer::Stop()
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
void SensorManager::SensorContainer::RunLoop()
{
  double minUpdateRate = GZ_DBL_MAX;

  // Get the minimum update rate from the sensors.
  for (Sensor_V::iterator iter = this->sensors.begin();
       iter != this->sensors.end(); ++iter)
  {
    minUpdateRate = std::min((*iter)->GetUpdateRate(), minUpdateRate);
  }

  std::cout << "MinUpdateRate[" << minUpdateRate << "]\n";
  minUpdateRate *= 2.0;

  common::Time sleepTime(1.0 / (minUpdateRate * 2.0));

  while (!this->stop)
  {
    this->Update(false);
    common::Time::Sleep(sleepTime);
  }
}

//////////////////////////////////////////////////
void SensorManager::SensorContainer::Update(bool _force)
{
  boost::recursive_mutex::scoped_lock lock(this->mutex);

  for (Sensor_V::iterator iter = this->sensors.begin();
       iter != this->sensors.end(); ++iter)
  {
    (*iter)->Update(_force);
  }
}

//////////////////////////////////////////////////
SensorPtr SensorManager::SensorContainer::GetSensor(const std::string &_name,
                                                    bool _useLeafName) const
{
  boost::recursive_mutex::scoped_lock lock(this->mutex);

  SensorPtr result;

  // Look for a sensor with the correct name
  for (Sensor_V::const_iterator iter = this->sensors.begin();
       iter != this->sensors.end() && !result; ++iter)
  {
    // We match on the scoped name (model::link::sensor) because multiple
    // sensors with the name leaf name make exists in a world.
    if ((_useLeafName && (*iter)->GetName() == _name) ||
        (!_useLeafName && (*iter)->GetScopedName() == _name) )
    {
      result = (*iter);
    }
  }

  return result;
}

//////////////////////////////////////////////////
void SensorManager::SensorContainer::AddSensor(SensorPtr _sensor)
{
  boost::recursive_mutex::scoped_lock lock(this->mutex);
  this->sensors.push_back(_sensor);
}

//////////////////////////////////////////////////
bool SensorManager::SensorContainer::RemoveSensor(const std::string &_name)
{
  boost::recursive_mutex::scoped_lock lock(this->mutex);

  Sensor_V::iterator iter;

  bool removed = false;

  // Find the correct sensor based on name, and remove it.
  for (iter = this->sensors.begin(); iter != this->sensors.end(); ++iter)
  {
    if ((*iter)->GetScopedName() == _name)
    {
      (*iter)->Fini();
      this->sensors.erase(iter);
      removed = true;
    }
  }

  return removed;
}

//////////////////////////////////////////////////
void SensorManager::SensorContainer::RemoveSensors()
{
  Sensor_V::iterator iter;

  // Remove all the sensors
  for (iter = this->sensors.begin(); iter != this->sensors.end(); ++iter)
    (*iter)->Fini();

  this->sensors.clear();
}

//////////////////////////////////////////////////
void SensorManager::ImageSensorContainer::Update(bool _force)
{
  event::Events::preRender();

  // Tell all the cameras to render
  event::Events::render();

  event::Events::postRender();

  // Update the sensors, which will produce data messages.
  SensorContainer::Update(_force);
}
