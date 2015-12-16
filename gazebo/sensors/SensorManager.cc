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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Time.hh"

#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/sensors/Sensor.hh"
#include "gazebo/sensors/SensorsIface.hh"
#include "gazebo/sensors/SensorFactory.hh"

#include "gazebo/sensors/SensorContainer.hh"
#include "gazebo/sensors/SensorManagerPrivate.hh"
#include "gazebo/sensors/SensorManager.hh"

using namespace gazebo;
using namespace sensors;

/// \brief A mutex used by SensorContainer and SimTimeEventHandler
/// for timing coordination.
std::mutex g_sensorTimingMutex;

//////////////////////////////////////////////////
SensorManager::SensorManager()
: dataPtr(new SensorManagerPrivate)
{
  // sensors::IMAGE container
  this->dataPtr->sensorContainers.push_back(new ImageSensorContainer());

  // sensors::RAY container
  this->dataPtr->sensorContainers.push_back(new SensorContainer());

  // sensors::OTHER container
  this->dataPtr->sensorContainers.push_back(new SensorContainer());
}

//////////////////////////////////////////////////
SensorManager::~SensorManager()
{
  // Clean up the sensors.
  for (SensorContainer_V::iterator iter =
       this->dataPtr->sensorContainers.begin();
       iter != this->dataPtr->sensorContainers.end(); ++iter)
  {
    GZ_ASSERT((*iter) != NULL, "Sensor Constainer is NULL");
    (*iter)->Stop();
    (*iter)->RemoveSensors();
    delete (*iter);
  }
  this->dataPtr->sensorContainers.clear();

  this->dataPtr->initSensors.clear();
}

//////////////////////////////////////////////////
void SensorManager::RunThreads()
{
  // Start the non-image sensor containers. The first item in the
  // sensorsContainers list are the image-based sensors, which rely on the
  // rendering engine, which in turn requires that they run in the main
  // thread.
  for (SensorContainer_V::iterator iter =
       ++this->dataPtr->sensorContainers.begin();
       iter != this->dataPtr->sensorContainers.end(); ++iter)
  {
    GZ_ASSERT((*iter) != NULL, "Sensor Constainer is NULL");
    (*iter)->Run();
  }
}

//////////////////////////////////////////////////
void SensorManager::Stop()
{
  // Start all the sensor containers.
  for (SensorContainer_V::iterator iter =
       this->dataPtr->sensorContainers.begin();
       iter != this->dataPtr->sensorContainers.end(); ++iter)
  {
    GZ_ASSERT((*iter) != NULL, "Sensor Constainer is NULL");
    (*iter)->Stop();
  }
}

//////////////////////////////////////////////////
void SensorManager::Update(const bool _force)
{
  {
    std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);

    // in case things are spawn, sensors length changes
    for (Sensor_V::iterator iter = this->initSensors.begin();
         iter != this->initSensors.end(); ++iter)
    {
      GZ_ASSERT((*iter) != NULL, "Sensor pointer is NULL");
      GZ_ASSERT((*iter)->GetCategory() < 0 ||
          (*iter)->GetCategory() < CATEGORY_COUNT, "Sensor category is empty");
      GZ_ASSERT(this->dataPtr->sensorContainers[
          (*iter)->GetCategory()] != NULL, "Sensor container is NULL");

      (*iter)->Init();
      this->dataPtr->sensorContainers[(*iter)->GetCategory()]->AddSensor(*iter);
    }
    this->initSensors.clear();

    for (std::vector<std::string>::iterator iter =
         this->removeSensors.begin(); iter != this->removeSensors.end(); ++iter)
    {
      GZ_ASSERT(!(*iter).empty(), "Remove sensor name is empty.");

      bool removed = false;
      for (SensorContainer_V::iterator iter2 =
           this->dataPtr->sensorContainers.begin();
           iter2 != this->dataPtr->sensorContainers.end() && !removed; ++iter2)
      {
        GZ_ASSERT((*iter2) != NULL, "SensorContainer is NULL");

        removed = (*iter2)->RemoveSensor(*iter);
      }

      if (!removed)
      {
        gzerr << "RemoveSensor failed. The SensorManager's list of sensors "
              << "changed during sensor removal. This is bad, and should "
              << "never happen.\n";
      }
    }
    this->removeSensors.clear();

    if (this->dataPtr->removeAllSensorss)
    {
      for (SensorContainer_V::iterator iter2 =
           this->dataPtr->sensorContainers.begin();
           iter2 != this->dataPtr->sensorContainers.end(); ++iter2)
      {
        GZ_ASSERT((*iter2) != NULL, "SensorContainer is NULL");
        (*iter2)->RemoveSensors();
      }
      this->initSensors.clear();
      this->dataPtr->removeAllSensorss = false;
    }
  }

  // Only update if there are sensors
  if (this->dataPtr->sensorContainers[sensors::IMAGE]->sensors.size() > 0)
    this->dataPtr->sensorContainers[sensors::IMAGE]->Update(_force);
}

//////////////////////////////////////////////////
bool SensorManager::SensorsInitialized()
{
  std::lock_guar<std::recursive_mutex> lock(this->dataPtr->mutex);
  bool result = this->initSensors.empty();
  return result;
}

//////////////////////////////////////////////////
void SensorManager::ResetLastUpdateTimes()
{
  std::lock_guard<recursive_mutex> lock(this->dataPtr->mutex);
  for (SensorContainer_V::iterator iter =
       this->dataPtr->sensorContainers.begin();
       iter != this->dataPtr->sensorContainers.end(); ++iter)
  {
    GZ_ASSERT((*iter) != NULL, "SensorContainer is NULL");
    (*iter)->ResetLastUpdateTimes();
  }
}

//////////////////////////////////////////////////
void SensorManager::Init()
{
  std::lock_guard<recursive_mutex> lock(this->dataPtr->mutex);

  this->dataPtr->simTimeEventHandler = new SimTimeEventHandler();

  // Initialize all the sensor containers.
  for (SensorContainer_V::iterator iter =
       this->dataPtr->sensorContainers.begin();
       iter != this->dataPtr->sensorContainers.end(); ++iter)
  {
    GZ_ASSERT((*iter) != NULL, "SensorContainer is NULL");
    (*iter)->Init();
  }

  this->dataPtr->initialized = true;
}

//////////////////////////////////////////////////
void SensorManager::Fini()
{
  std::lock_guard<recursive_mutex> lock(this->dataPtr->mutex);

  // Finalize all the sensor containers.
  for (SensorContainer_V::iterator iter =
       this->dataPtr->sensorContainers.begin();
       iter != this->dataPtr->sensorContainers.end(); ++iter)
  {
    GZ_ASSERT((*iter) != NULL, "SensorContainer is NULL");
    (*iter)->Fini();
    (*iter)->Stop();
  }

  this->dataPtr->removeSensors.clear();

  delete this->dataPtr->simTimeEventHandler;
  this->dataPtr->simTimeEventHandler = NULL;

  this->dataPtr->initialized = false;
}

//////////////////////////////////////////////////
void SensorManager::GetSensorTypes(std::vector<std::string> &_types) const
{
  return this->SensorTypes(_types);
}

//////////////////////////////////////////////////
void SensorManager::SensorTypes(std::vector<std::string> &_types) const
{
  sensors::SensorFactory::SensorTypes(_types);
}

//////////////////////////////////////////////////
std::string SensorManager::CreateSensor(sdf::ElementPtr _elem,
                                        const std::string &_worldName,
                                        const std::string &_parentName,
                                        const uint32_t _parentId)
{
  std::string type = _elem->Get<std::string>("type");
  SensorPtr sensor = sensors::SensorFactory::NewSensor(type);

  if (!sensor)
  {
    gzerr << "Unable to create sensor of type[" << type << "]\n";
    return std::string();
  }

  // Must come before sensor->Load
  sensor->SetParent(_parentName, _parentId);

  // Load the sensor
  sensor->Load(_worldName, _elem);

  // If the SensorManager has not been initialized, then it's okay to push
  // the sensor into one of the sensor vectors because the sensor will get
  // initialized in SensorManager::Init
  if (!this->dataPtr->initialized)
  {
    this->dataPtr->sensorContainers[sensor->GetCategory()]->AddSensor(sensor);
  }
  // Otherwise the SensorManager is already running, and the sensor will get
  // initialized during the next SensorManager::Update call.
  else
  {
    std::lock_guard<recursive_mutex> lock(this->dataPtr->mutex);
    this->initSensors.push_back(sensor);
  }

  return sensor->GetScopedName();
}

//////////////////////////////////////////////////
SensorPtr SensorManager::GetSensor(const std::string &_name) const
{
  return this->Sensor(_name);
}

//////////////////////////////////////////////////
SensorPtr SensorManager::Sensor(const std::string &_name) const
{
  std::lock_guard<recursive_mutex> lock(this->dataPtr->mutex);

  SensorContainer_V::const_iterator iter;
  SensorPtr result;

  // Try to find the sensor in all of the containers
  for (iter = this->dataPtr->sensorContainers.begin();
       iter != this->dataPtr->sensorContainers.end() && !result; ++iter)
  {
    GZ_ASSERT((*iter) != NULL, "SensorContainer is NULL");
    result = (*iter)->GetSensor(_name);
  }

  // If the sensor was not found, then try to find based on an unscoped
  // name.
  // If multiple sensors exist with the same name, then an error occurs
  // because we don't know which sensor is correct.
  if (!result)
  {
    SensorPtr tmpSensor;
    for (iter = this->dataPtr->sensorContainers.begin();
         iter != this->dataPtr->sensorContainers.end(); ++iter)
    {
      GZ_ASSERT((*iter) != NULL, "SensorContainer is NULL");
      tmpSensor = (*iter)->GetSensor(_name, true);

      if (!tmpSensor)
        continue;

      if (!result)
      {
        result = tmpSensor;
        GZ_ASSERT(result != NULL, "SensorContainer contains a NULL Sensor");
      }
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
  return this->Sensors();
}

//////////////////////////////////////////////////
Sensor_V SensorManager::Sensors() const
{
  std::lock_guard<recursive_mutex> lock(this->dataPtr->mutex);
  Sensor_V result;

  // Copy the sensor pointers
  for (SensorContainer_V::const_iterator iter = this->dataPtr->sensorContainers.begin();
       iter != this->dataPtr->sensorContainers.end(); ++iter)
  {
    GZ_ASSERT((*iter) != NULL, "SensorContainer is NULL");
    std::copy((*iter)->sensors.begin(), (*iter)->sensors.end(),
              std::back_inserter(result));
  }

  return result;
}

//////////////////////////////////////////////////
void SensorManager::RemoveSensor(const std::string &_name)
{
  std::lock_guar<std::recursive_mutex> lock(this->dataPtr->mutex);
  SensorPtr sensor = this->GetSensor(_name);

  if (!sensor)
  {
    gzerr << "Unable to remove sensor[" << _name << "] because it "
          << "does not exist.\n";
  }
  else
  {
    // Push it on the list, to be removed by the main sensor thread,
    // to ensure correct access to rendering resources.
    this->removeSensors.push_back(sensor->GetScopedName());
  }
}

//////////////////////////////////////////////////
void SensorManager::RemoveSensors()
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
  this->dataPtr->removeAllSensorss = true;
}



/////////////////////////////////////////////////
SimTimeEventHandler::SimTimeEventHandler()
{
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&SimTimeEventHandler::OnUpdate, this,
        std::placeholders::_1));
}

/////////////////////////////////////////////////
SimTimeEventHandler::~SimTimeEventHandler()
{
  // Cleanup the events.
  for (std::list<SimTimeEvent*>::iterator iter = this->events.begin();
       iter != this->events.end(); ++iter)
  {
    GZ_ASSERT(*iter != NULL, "SimTimeEvent is NULL");
    delete *iter;
  }
  this->events.clear();
}

/////////////////////////////////////////////////
void SimTimeEventHandler::AddRelativeEvent(const common::Time &_time,
                                           std::condition_variable *_var)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  physics::WorldPtr world = physics::get_world();
  GZ_ASSERT(world != NULL, "World pointer is NULL");

  // Create the new event.
  SimTimeEvent *event = new SimTimeEvent;
  event->time = world->GetSimTime() + _time;
  event->condition = _var;

  // Add the event to the list.
  this->events.push_back(event);
}

/////////////////////////////////////////////////
void SimTimeEventHandler::OnUpdate(const common::UpdateInfo &_info)
{
  std::lock_guard<std::mutex> timingLock(g_sensorTimingMutex);
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Iterate over all the events.
  for (std::list<SimTimeEvent*>::iterator iter = this->events.begin();
      iter != this->events.end();)
  {
    GZ_ASSERT(*iter != NULL, "SimTimeEvent is NULL");

    // Find events that have a time less than or equal to simulation
    // time.
    if ((*iter)->time <= _info.simTime)
    {
      // Notify the event by triggering its condition.
      (*iter)->condition->notify_all();

      // Remove the event.
      delete *iter;
      this->events.erase(iter++);
    }
    else
      ++iter;
  }
}
