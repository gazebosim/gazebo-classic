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
/*
 * Desc: Class to manager all sensors
 * Author: Nate Koenig
 * Date: 18 Dec 2009
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
#include "gazebo/sensors/SensorManager.hh"

using namespace gazebo;
using namespace sensors;

/// \brief A mutex used by SensorContainer and SimTimeEventHandler
/// for timing coordination.
boost::mutex g_sensorTimingMutex;

//////////////////////////////////////////////////
SensorManager::SensorManager()
  : initialized(false), removeAllSensors(false)
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
    GZ_ASSERT((*iter) != NULL, "Sensor Constainer is NULL");
    (*iter)->Stop();
    (*iter)->RemoveSensors();
    delete (*iter);
  }
  this->sensorContainers.clear();

  this->initSensors.clear();
}

//////////////////////////////////////////////////
void SensorManager::RunThreads()
{
  // Start the non-image sensor containers. The first item in the
  // sensorsContainers list are the image-based sensors, which rely on the
  // rendering engine, which in turn requires that they run in the main
  // thread.
  for (SensorContainer_V::iterator iter = ++this->sensorContainers.begin();
       iter != this->sensorContainers.end(); ++iter)
  {
    GZ_ASSERT((*iter) != NULL, "Sensor Constainer is NULL");
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
    GZ_ASSERT((*iter) != NULL, "Sensor Constainer is NULL");
    (*iter)->Stop();
  }
}

//////////////////////////////////////////////////
void SensorManager::Update(bool _force)
{
  {
    boost::recursive_mutex::scoped_lock lock(this->mutex);

    // in case things are spawn, sensors length changes
    for (Sensor_V::iterator iter = this->initSensors.begin();
         iter != this->initSensors.end(); ++iter)
    {
      GZ_ASSERT((*iter) != NULL, "Sensor pointer is NULL");
      GZ_ASSERT((*iter)->GetCategory() < 0 ||
          (*iter)->GetCategory() < CATEGORY_COUNT, "Sensor category is empty");
      GZ_ASSERT(this->sensorContainers[(*iter)->GetCategory()] != NULL,
                "Sensor container is NULL");

      (*iter)->Init();
      this->sensorContainers[(*iter)->GetCategory()]->AddSensor(*iter);
    }
    this->initSensors.clear();

    for (std::vector<std::string>::iterator iter = this->removeSensors.begin();
         iter != this->removeSensors.end(); ++iter)
    {
      GZ_ASSERT(!(*iter).empty(), "Remove sensor name is empty.");

      bool removed = false;
      for (SensorContainer_V::iterator iter2 = this->sensorContainers.begin();
           iter2 != this->sensorContainers.end() && !removed; ++iter2)
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

    if (this->removeAllSensors)
    {
      for (SensorContainer_V::iterator iter2 = this->sensorContainers.begin();
          iter2 != this->sensorContainers.end(); ++iter2)
      {
        GZ_ASSERT((*iter2) != NULL, "SensorContainer is NULL");
        (*iter2)->RemoveSensors();
      }
      this->initSensors.clear();
      this->removeAllSensors = false;
    }
  }

  // Only update if there are sensors
  if (this->sensorContainers[sensors::IMAGE]->sensors.size() > 0)
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
void SensorManager::ResetLastUpdateTimes()
{
  boost::recursive_mutex::scoped_lock lock(this->mutex);
  for (SensorContainer_V::iterator iter = this->sensorContainers.begin();
       iter != this->sensorContainers.end(); ++iter)
  {
    GZ_ASSERT((*iter) != NULL, "SensorContainer is NULL");
    (*iter)->ResetLastUpdateTimes();
  }
}

//////////////////////////////////////////////////
void SensorManager::Init()
{
  boost::recursive_mutex::scoped_lock lock(this->mutex);

  this->simTimeEventHandler = new SimTimeEventHandler();

  // Initialize all the sensor containers.
  for (SensorContainer_V::iterator iter = this->sensorContainers.begin();
       iter != this->sensorContainers.end(); ++iter)
  {
    GZ_ASSERT((*iter) != NULL, "SensorContainer is NULL");
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
    GZ_ASSERT((*iter) != NULL, "SensorContainer is NULL");
    (*iter)->Fini();
    (*iter)->Stop();
  }

  this->removeSensors.clear();

  delete this->simTimeEventHandler;
  this->simTimeEventHandler = NULL;

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
                                        const std::string &_parentName,
                                        uint32_t _parentId)
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
    for (iter = this->sensorContainers.begin();
         iter != this->sensorContainers.end(); ++iter)
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
  boost::recursive_mutex::scoped_lock lock(this->mutex);
  Sensor_V result;

  // Copy the sensor pointers
  for (SensorContainer_V::const_iterator iter = this->sensorContainers.begin();
       iter != this->sensorContainers.end(); ++iter)
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
  boost::recursive_mutex::scoped_lock lock(this->mutex);
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
  boost::recursive_mutex::scoped_lock lock(this->mutex);
  this->removeAllSensors = true;
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
    GZ_ASSERT((*iter) != NULL, "Sensor is NULL");
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
    GZ_ASSERT((*iter) != NULL, "Sensor is NULL");
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

  GZ_ASSERT(this->runThread, "Unable to create boost::thread.");
}

//////////////////////////////////////////////////
void SensorManager::SensorContainer::Stop()
{
  this->stop = true;
  this->runCondition.notify_all();
  if (this->runThread)
  {
    // Note: calling interrupt seems to cause the thread to either block
    // or throw an exception, so commenting it out for now.
    // this->runThread->interrupt();
    this->runThread->join();
    delete this->runThread;
    this->runThread = NULL;
  }
}

//////////////////////////////////////////////////
void SensorManager::SensorContainer::RunLoop()
{
  this->stop = false;

  physics::WorldPtr world = physics::get_world();
  GZ_ASSERT(world != NULL, "Pointer to World is NULL");

  physics::PhysicsEnginePtr engine = world->GetPhysicsEngine();
  GZ_ASSERT(engine != NULL, "Pointer to PhysicsEngine is NULL");

  engine->InitForThread();

  common::Time sleepTime, startTime, eventTime, diffTime;
  double maxUpdateRate = 0;

  boost::mutex tmpMutex;
  boost::mutex::scoped_lock lock2(tmpMutex);

  // Wait for a sensor to be added.
  // Use a while loop since world resets will notify the runCondition.
  while (this->sensors.empty())
  {
    this->runCondition.wait(lock2);
    if (this->stop)
      return;
  }

  {
    boost::recursive_mutex::scoped_lock lock(this->mutex);

    // Get the minimum update rate from the sensors.
    for (Sensor_V::iterator iter = this->sensors.begin();
        iter != this->sensors.end() && !this->stop; ++iter)
    {
      GZ_ASSERT((*iter) != NULL, "Sensor is NULL");
      maxUpdateRate = std::max((*iter)->GetUpdateRate(), maxUpdateRate);
    }
  }

  // Calculate an appropriate sleep time.
  if (maxUpdateRate > 0)
    sleepTime.Set(1.0 / (maxUpdateRate));
  else
    sleepTime.Set(0, 1e6);

  while (!this->stop)
  {
    // If all the sensors get deleted, wait here.
    // Use a while loop since world resets will notify the runCondition.
    while (this->sensors.empty())
    {
      this->runCondition.wait(lock2);
      if (this->stop)
        return;
    }

    // Get the start time of the update.
    startTime = world->GetSimTime();

    this->Update(false);

    // Compute the time it took to update the sensors.
    // It's possible that the world time was reset during the Update. This
    // would case a negative diffTime. Instead, just use a event time of zero
    diffTime = std::max(common::Time::Zero, world->GetSimTime() - startTime);

    // Set the default sleep time
    eventTime = std::max(common::Time::Zero, sleepTime - diffTime);

    // Make sure update time is reasonable.
    GZ_ASSERT(diffTime.sec < 1, "Took over 1.0 seconds to update a sensor.");

    // Make sure eventTime is not negative.
    GZ_ASSERT(eventTime >= common::Time::Zero,
        "Time to next sensor update is negative.");

    boost::mutex::scoped_lock timingLock(g_sensorTimingMutex);

    // Add an event to trigger when the appropriate simulation time has been
    // reached.
    SensorManager::Instance()->simTimeEventHandler->AddRelativeEvent(
        eventTime, &this->runCondition);

    // This if statement helps prevent deadlock on osx during teardown.
    if (!this->stop)
    {
      this->runCondition.wait(timingLock);
    }
  }
}

//////////////////////////////////////////////////
void SensorManager::SensorContainer::Update(bool _force)
{
  boost::recursive_mutex::scoped_lock lock(this->mutex);

  if (this->sensors.empty())
    gzlog << "Updating a sensor container without any sensors.\n";

  // Update all the sensors in this container.
  for (Sensor_V::iterator iter = this->sensors.begin();
       iter != this->sensors.end(); ++iter)
  {
    GZ_ASSERT((*iter) != NULL, "Sensor is NULL");
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
    GZ_ASSERT((*iter) != NULL, "Sensor is NULL");

    // We match on the scoped name (model::link::sensor) because multiple
    // sensors with the name leaf name make exists in a world.
    if ((_useLeafName && (*iter)->GetName() == _name) ||
        (!_useLeafName && (*iter)->GetScopedName() == _name))
    {
      result = (*iter);
      break;
    }
  }

  return result;
}

//////////////////////////////////////////////////
void SensorManager::SensorContainer::AddSensor(SensorPtr _sensor)
{
  GZ_ASSERT(_sensor != NULL, "Sensor is NULL when passed to ::AddSensor");

  {
    boost::recursive_mutex::scoped_lock lock(this->mutex);
    this->sensors.push_back(_sensor);
  }

  // Tell the run loop that we have received a sensor
  this->runCondition.notify_one();
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
    GZ_ASSERT((*iter) != NULL, "Sensor is NULL");

    if ((*iter)->GetScopedName() == _name)
    {
      (*iter)->Fini();
      this->sensors.erase(iter);
      removed = true;
      break;
    }
  }

  return removed;
}

//////////////////////////////////////////////////
void SensorManager::SensorContainer::ResetLastUpdateTimes()
{
  boost::recursive_mutex::scoped_lock lock(this->mutex);

  Sensor_V::iterator iter;

  // Rest last update times for all contained sensors.
  for (iter = this->sensors.begin(); iter != this->sensors.end(); ++iter)
  {
    GZ_ASSERT((*iter) != NULL, "Sensor is NULL");
    (*iter)->ResetLastUpdateTime();
  }

  // Tell the run loop that world time has been reset.
  this->runCondition.notify_one();
}

//////////////////////////////////////////////////
void SensorManager::SensorContainer::RemoveSensors()
{
  boost::recursive_mutex::scoped_lock lock(this->mutex);

  Sensor_V::iterator iter;

  // Remove all the sensors
  for (iter = this->sensors.begin(); iter != this->sensors.end(); ++iter)
  {
    GZ_ASSERT((*iter) != NULL, "Sensor is NULL");
    (*iter)->Fini();
  }

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




/////////////////////////////////////////////////
SimTimeEventHandler::SimTimeEventHandler()
{
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&SimTimeEventHandler::OnUpdate, this, _1));
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
                                           boost::condition_variable *_var)
{
  boost::mutex::scoped_lock lock(this->mutex);

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
  boost::mutex::scoped_lock timingLock(g_sensorTimingMutex);
  boost::mutex::scoped_lock lock(this->mutex);

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
