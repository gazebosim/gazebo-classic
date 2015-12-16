/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

using namespace gazebo;
using namespace sensors;

//////////////////////////////////////////////////
SensorContainer::SensorContainer()
{
  this->stop = true;
  this->dataPtr->initialized = false;
  this->runThread = NULL;
}

//////////////////////////////////////////////////
SensorContainer::~SensorContainer()
{
  this->sensors.clear();
}

//////////////////////////////////////////////////
void SensorContainer::Init()
{
  boost::recursive_mutex::scoped_lock lock(this->dataPtr->mutex);

  Sensor_V::iterator iter;
  for (iter = this->sensors.begin(); iter != this->sensors.end(); ++iter)
  {
    GZ_ASSERT((*iter) != NULL, "Sensor is NULL");
    (*iter)->Init();
  }

  this->dataPtr->initialized = true;
}

//////////////////////////////////////////////////
void SensorContainer::Fini()
{
  boost::recursive_mutex::scoped_lock lock(this->dataPtr->mutex);

  Sensor_V::iterator iter;

  // Finialize each sensor in the current sensor vector
  for (iter = this->sensors.begin(); iter != this->sensors.end(); ++iter)
  {
    GZ_ASSERT((*iter) != NULL, "Sensor is NULL");
    (*iter)->Fini();
  }

  // Remove all the sensors from the current sensor vector.
  this->sensors.clear();

  this->dataPtr->initialized = false;
}

//////////////////////////////////////////////////
void SensorContainer::Run()
{
  this->runThread = new boost::thread(
      boost::bind(&SensorContainer::RunLoop, this));

  GZ_ASSERT(this->runThread, "Unable to create boost::thread.");
}

//////////////////////////////////////////////////
void SensorContainer::Stop()
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
void SensorContainer::RunLoop()
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
    boost::recursive_mutex::scoped_lock lock(this->dataPtr->mutex);

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
void SensorContainer::Update(bool _force)
{
  boost::recursive_mutex::scoped_lock lock(this->dataPtr->mutex);

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
SensorPtr SensorContainer::GetSensor(const std::string &_name,
                                                    bool _useLeafName) const
{
  boost::recursive_mutex::scoped_lock lock(this->dataPtr->mutex);

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
void SensorContainer::AddSensor(SensorPtr _sensor)
{
  GZ_ASSERT(_sensor != NULL, "Sensor is NULL when passed to ::AddSensor");

  {
    boost::recursive_mutex::scoped_lock lock(this->dataPtr->mutex);
    this->sensors.push_back(_sensor);
  }

  // Tell the run loop that we have received a sensor
  this->runCondition.notify_one();
}

//////////////////////////////////////////////////
bool SensorContainer::RemoveSensor(const std::string &_name)
{
  boost::recursive_mutex::scoped_lock lock(this->dataPtr->mutex);

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
void SensorContainer::ResetLastUpdateTimes()
{
  boost::recursive_mutex::scoped_lock lock(this->dataPtr->mutex);

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
void SensorContainer::RemoveSensors()
{
  boost::recursive_mutex::scoped_lock lock(this->dataPtr->mutex);

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
