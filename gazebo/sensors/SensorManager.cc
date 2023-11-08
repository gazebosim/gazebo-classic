/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include <functional>
#include <boost/bind/bind.hpp>

#include "gazebo/physics/Link.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/sensors/CameraSensor.hh"
#include "gazebo/sensors/Sensor.hh"
#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/sensors/SensorsIface.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/util/LogPlay.hh"

#include "ignition/common/Profiler.hh"

using namespace gazebo;
using namespace sensors;

/// \brief A mutex used by SensorContainer and SimTimeEventHandler
/// for timing coordination.
boost::mutex g_sensorTimingMutex;

/// \brief Flag to indicate if number of sensors has changed and that the
/// max update rate needs to be recalculated
bool g_sensorsDirty = true;

/// Performance metrics variables
/// \brief last sensor measurement sim time
std::map<std::string, gazebo::common::Time> sensorsLastMeasurementTime;

/// \brief last sensor measurement real time
std::map<std::string, gazebo::common::Time> worldLastMeasurementTime;

/// \brief Data structure for storing metric data to be published
struct sensorPerformanceMetricsType
{
  /// \brief sensor real time update rate
  double sensorRealUpdateRate;

  /// \brief sensor sim time update rate
  double sensorSimUpdateRate;

  /// \brief Rendering sensor average real time update rate. The difference
  /// between sensorAvgFps and sensorRealUpdateRte is that sensorAvgFps is
  /// for rendering sensors only and the rate is averaged over a fixed
  /// window size, whereas the sensorSimUpdateRate stores the instantaneous
  /// update rate and it is filled by all sensors.
  double sensorAvgFPS;
};

/// \brief A map of sensor name to its performance metrics data
std::map<std::string, struct sensorPerformanceMetricsType>
    sensorPerformanceMetrics;

/// \brief Publisher for run-time simulation performance metrics.
transport::PublisherPtr performanceMetricsPub;

/// \brief Node for publishing performance metrics
transport::NodePtr node = nullptr;

/// \brief Last sim time measured for performance metrics
common::Time lastSimTime;

/// \brief Last real time measured for performance metrics
common::Time lastRealTime;

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
  sensors::disable();
  // Clean up the sensors.
  for (SensorContainer_V::iterator iter = this->sensorContainers.begin();
       iter != this->sensorContainers.end(); ++iter)
  {
    GZ_ASSERT((*iter) != nullptr, "Sensor Constainer is null");
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
    GZ_ASSERT((*iter) != nullptr, "Sensor Constainer is null");
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
    GZ_ASSERT((*iter) != nullptr, "Sensor Constainer is null");
    (*iter)->Stop();
  }

  if (!physics::worlds_running())
    this->worlds.clear();
}

//////////////////////////////////////////////////
bool SensorManager::Running() const
{
  for (auto const &container : this->sensorContainers)
  {
    if (container->Running())
      return true;
  }
  return false;
}

//////////////////////////////////////////////////
void SensorManager::WaitForSensors(double _clk, double _dt)
{
  double tnext = this->NextRequiredTimestamp();

  while (!std::isnan(tnext)
      && ignition::math::lessOrNearEqual(tnext - _dt / 2.0, _clk)
      && physics::worlds_running())
  {
    this->WaitForPrerendered(0.001);
    tnext = this->NextRequiredTimestamp();
  }
}

void PublishPerformanceMetrics()
{
  if (node == nullptr)
  {
    auto world = physics::get_world();
    // Transport
    node = transport::NodePtr(new transport::Node());
    node->Init(world->Name());
    performanceMetricsPub =
     node->Advertise<msgs::PerformanceMetrics>(
         "/gazebo/performance_metrics", 10, 5);
  }

  if (!performanceMetricsPub || !performanceMetricsPub->HasConnections())
  {
    return;
  }

  physics::WorldPtr world = physics::get_world(
    gazebo::physics::get_world()->Name());

  /// Outgoing run-time simulation performance metrics.
  msgs::PerformanceMetrics performanceMetricsMsg;

  // Real time factor
  common::Time realTime = world->RealTime();
  common::Time diffRealtime = realTime - lastRealTime;
  common::Time simTime = world->SimTime();
  common::Time diffSimTime = simTime - lastSimTime;
  common::Time realTimeFactor;

  if (ignition::math::equal(diffSimTime.Double(), 0.0))
    return;

  if (diffRealtime == common::Time::Zero)
    realTimeFactor = 0;
  else
    realTimeFactor = diffSimTime / diffRealtime;

  performanceMetricsMsg.set_real_time_factor(realTimeFactor.Double());

  lastRealTime = realTime;
  lastSimTime = simTime;

  /// update sim time for sensors
  for (auto model: world->Models())
  {
    for (auto link: model->GetLinks())
    {
      for (unsigned int i = 0; i < link->GetSensorCount(); i++)
      {
        std::string name = link->GetSensorName(i);
        sensors::SensorPtr sensor = sensors::get_sensor(name);

        auto ret = sensorsLastMeasurementTime.insert(
            std::pair<std::string, gazebo::common::Time>(name, 0));
        worldLastMeasurementTime.insert(
                std::pair<std::string, gazebo::common::Time>(name, 0));
        if (ret.second == false)
        {
          double updateSimRate =
            (sensor->LastMeasurementTime() - sensorsLastMeasurementTime[name])
            .Double();
          if (updateSimRate > 0.0)
          {
            sensorsLastMeasurementTime[name] = sensor->LastMeasurementTime();
            double updateRealRate =
              (world->RealTime() - worldLastMeasurementTime[name]).Double();

            struct sensorPerformanceMetricsType emptySensorPerfomanceMetrics;
            auto ret2 = sensorPerformanceMetrics.insert(
                std::pair<std::string, struct sensorPerformanceMetricsType>
                  (name, emptySensorPerfomanceMetrics));
            if (ret2.second == false)
            {
              ret2.first->second.sensorSimUpdateRate =
                  1.0/updateSimRate;
              ret2.first->second.sensorRealUpdateRate =
                  1.0/updateRealRate;
              worldLastMeasurementTime[name] = world->RealTime();

              // Special case for stereo cameras
              sensors::CameraSensorPtr cameraSensor =
                std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
              if (nullptr != cameraSensor)
              {
                ret2.first->second.sensorAvgFPS =
                    cameraSensor->Camera()->AvgFPS();
              }
              else
              {
                ret2.first->second.sensorAvgFPS = -1;
              }
            }
          }
        }
      }
    }
  }

  for (auto sensorPerformanceMetric: sensorPerformanceMetrics)
  {
    msgs::PerformanceMetrics::PerformanceSensorMetrics
        *performanceSensorMetricsMsg = performanceMetricsMsg.add_sensor();
    performanceSensorMetricsMsg->set_name(sensorPerformanceMetric.first);
    performanceSensorMetricsMsg->set_real_update_rate(
      sensorPerformanceMetric.second.sensorRealUpdateRate);
    performanceSensorMetricsMsg->set_sim_update_rate(
      sensorPerformanceMetric.second.sensorSimUpdateRate);
    if (sensorPerformanceMetric.second.sensorAvgFPS >= 0.0)
    {
      performanceSensorMetricsMsg->set_fps(
        sensorPerformanceMetric.second.sensorAvgFPS);
    }
  }

  // Publish data
  performanceMetricsPub->Publish(performanceMetricsMsg);
}

//////////////////////////////////////////////////
void SensorManager::Update(bool _force)
{
  {
    boost::recursive_mutex::scoped_lock lock(this->mutex);

    if (this->worlds.empty() && physics::worlds_running() && this->initialized)
    {
      auto world = physics::get_world();
      this->worlds[world->Name()] = world;
      world->_SetSensorsInitialized(true);
    }

    if (!this->initSensors.empty())
    {
      // in case things are spawned, sensors length changes
      for (auto &sensor : this->initSensors)
      {
        GZ_ASSERT(sensor != nullptr, "Sensor pointer is null");
        GZ_ASSERT(sensor->Category() < 0 ||
            sensor->Category() < CATEGORY_COUNT, "Sensor category is empty");
        GZ_ASSERT(this->sensorContainers[sensor->Category()] != nullptr,
            "Sensor container is null");

        sensor->Init();
        this->sensorContainers[sensor->Category()]->AddSensor(sensor);
      }
      this->initSensors.clear();
      for (auto &worldName_worldPtr : this->worlds)
        worldName_worldPtr.second->_SetSensorsInitialized(true);
    }

    for (std::vector<std::string>::iterator iter = this->removeSensors.begin();
         iter != this->removeSensors.end(); ++iter)
    {
      GZ_ASSERT(!(*iter).empty(), "Remove sensor name is empty.");

      bool removed = false;
      for (SensorContainer_V::iterator iter2 = this->sensorContainers.begin();
           iter2 != this->sensorContainers.end() && !removed; ++iter2)
      {
        GZ_ASSERT((*iter2) != nullptr, "SensorContainer is null");

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
        GZ_ASSERT((*iter2) != nullptr, "SensorContainer is null");
        (*iter2)->RemoveSensors();
      }
      this->initSensors.clear();

      // Also clear the list of worlds
      this->worlds.clear();

      this->removeAllSensors = false;
    }
  }

  // Only update if there are sensors
  if (this->sensorContainers[sensors::IMAGE]->sensors.size() > 0)
    this->sensorContainers[sensors::IMAGE]->Update(_force);

  PublishPerformanceMetrics();
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
    GZ_ASSERT((*iter) != nullptr, "SensorContainer is null");
    (*iter)->ResetLastUpdateTimes();
  }
}

//////////////////////////////////////////////////
double SensorManager::NextRequiredTimestamp()
{
  double rv = std::numeric_limits<double>::quiet_NaN();

  // scan all sensors whose category is IMAGE
  for (auto& s : this->sensorContainers[sensors::IMAGE]->sensors)
  {
    // skip deactivated sensors
    if (!s->IsActive()) continue;

    double candidate = s->NextRequiredTimestamp();
    // take the smallest valid value
    if (!std::isnan(candidate)
        && (std::isnan(rv) || rv > candidate))
      rv = candidate;
  }

  return rv;
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
    GZ_ASSERT((*iter) != nullptr, "SensorContainer is null");
    (*iter)->Init();
  }

  // Connect to the time reset event.
  this->timeResetConnection = event::Events::ConnectTimeReset(
      std::bind(&SensorManager::ResetLastUpdateTimes, this));

  // Connect to the remove sensor event.
  this->removeSensorConnection = event::Events::ConnectRemoveSensor(
      std::bind(&SensorManager::RemoveSensor, this, std::placeholders::_1));

  // Connect to the create sensor event.
  this->createSensorConnection = event::Events::ConnectCreateSensor(
      std::bind(&SensorManager::OnCreateSensor, this,
        std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3, std::placeholders::_4));

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
    GZ_ASSERT((*iter) != nullptr, "SensorContainer is null");
    (*iter)->Fini();
    (*iter)->Stop();
  }

  this->removeSensors.clear();
  this->initSensors.clear();
  this->worlds.clear();

  delete this->simTimeEventHandler;
  this->simTimeEventHandler = nullptr;

  this->initialized = false;
}

//////////////////////////////////////////////////
void SensorManager::GetSensorTypes(std::vector<std::string> &_types) const
{
  sensors::SensorFactory::GetSensorTypes(_types);
}

//////////////////////////////////////////////////
void SensorManager::OnCreateSensor(sdf::ElementPtr _elem,
    const std::string &_worldName,
    const std::string &_parentName,
    const uint32_t _parentId)
{
  this->CreateSensor(_elem, _worldName, _parentName, _parentId);
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
  this->worlds[_worldName] = physics::get_world(_worldName);

  // Provide the wait function to the given world
  if (sensor->StrictRate())
    this->worlds[_worldName]->SetSensorWaitFunc(
        std::bind(&SensorManager::WaitForSensors, this,
          std::placeholders::_1, std::placeholders::_2));

  // If the SensorManager has not been initialized, then it's okay to push
  // the sensor into one of the sensor vectors because the sensor will get
  // initialized in SensorManager::Init
  if (!this->initialized)
  {
    this->sensorContainers[sensor->Category()]->AddSensor(sensor);
  }
  // Otherwise the SensorManager is already running, and the sensor will get
  // initialized during the next SensorManager::Update call.
  else
  {
    boost::recursive_mutex::scoped_lock lock(this->mutex);
    this->worlds[_worldName]->_SetSensorsInitialized(false);
    this->initSensors.push_back(sensor);
  }

  return sensor->ScopedName();
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
    GZ_ASSERT((*iter) != nullptr, "SensorContainer is null");
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
      GZ_ASSERT((*iter) != nullptr, "SensorContainer is null");
      tmpSensor = (*iter)->GetSensor(_name, true);

      if (!tmpSensor)
        continue;

      if (!result)
      {
        result = tmpSensor;
        GZ_ASSERT(result != nullptr,
            "SensorContainer contains a nullptr Sensor");
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
    GZ_ASSERT((*iter) != nullptr, "SensorContainer is null");
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
    this->removeSensors.push_back(sensor->ScopedName());
  }
}

//////////////////////////////////////////////////
bool SensorManager::WaitForPrerendered(double _timeoutsec)
{
  if (this->sensorContainers[sensors::IMAGE]->sensors.size() > 0)
    return ((ImageSensorContainer*)this->sensorContainers[sensors::IMAGE])
                ->WaitForPrerendered(_timeoutsec);

  return true;
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
  this->runThread = nullptr;
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

  for (auto &sensor : this->sensors)
  {
    GZ_ASSERT(sensor != nullptr, "Sensor is null");
    sensor->Init();
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
    GZ_ASSERT((*iter) != nullptr, "Sensor is null");
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
    this->runThread = nullptr;
  }
}

//////////////////////////////////////////////////
bool SensorManager::SensorContainer::Running() const
{
  return !this->stop;
}

//////////////////////////////////////////////////
void SensorManager::SensorContainer::RunLoop()
{
  this->stop = false;

  physics::WorldPtr world = physics::get_world();
  GZ_ASSERT(world != nullptr, "Pointer to World is null");

  physics::PhysicsEnginePtr engine = world->Physics();
  GZ_ASSERT(engine != nullptr, "Pointer to PhysicsEngine is null");

  engine->InitForThread();

  // The original value was hardcode to 1.0. Changed the value to
  // 1000 * MaxStepSize in order to handle simulation with a
  // large step size.
  double maxSensorUpdate = engine->GetMaxStepSize() * 1000;

  // Release engine pointer, we don't need it in the loop
  engine.reset();

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


  auto computeMaxUpdateRate = [&]()
  {
    {
      boost::recursive_mutex::scoped_lock lock(this->mutex);

      if (!g_sensorsDirty)
        return;

      // Get the maximum update rate from the sensors.
      for (Sensor_V::iterator iter = this->sensors.begin();
          iter != this->sensors.end() && !this->stop; ++iter)
      {
        GZ_ASSERT((*iter) != nullptr, "Sensor is null");
        maxUpdateRate = std::max((*iter)->UpdateRate(), maxUpdateRate);
      }

      g_sensorsDirty = false;
    }

    // Calculate an appropriate sleep time.
    if (maxUpdateRate > 0)
      sleepTime.Set(1.0 / (maxUpdateRate));
    else
      sleepTime.Set(0, 1e6);
  };

  computeMaxUpdateRate();

  IGN_PROFILE_THREAD_NAME("SensorManager");

  while (!this->stop)
  {
    IGN_PROFILE("SensorManager::RunLoop");

    // If all the sensors get deleted, wait here.
    // Use a while loop since world resets will notify the runCondition.
    while (this->sensors.empty())
    {
      this->runCondition.wait(lock2);
      if (this->stop)
        return;
    }

    computeMaxUpdateRate();

    // Get the start time of the update.
    startTime = world->SimTime();

    IGN_PROFILE_BEGIN("UpdateSensors");
    this->Update(false);
    IGN_PROFILE_END();

    // Compute the time it took to update the sensors.
    // It's possible that the world time was reset during the Update. This
    // would case a negative diffTime. Instead, just use a event time of zero
    diffTime = std::max(common::Time::Zero, world->SimTime() - startTime);

    // Set the default sleep time
    eventTime = std::max(common::Time::Zero, sleepTime - diffTime);

    // Make sure update time is reasonable.
    // During log playback, time can jump forward an arbitrary amount.
    if (diffTime.sec >= maxSensorUpdate && !util::LogPlay::Instance()->IsOpen())
    {
      gzwarn << "Took over 1000*max_step_size to update a sensor "
        << "(took " << diffTime.sec << " sec, which is more than "
        << "the max update of " << maxSensorUpdate << " sec). "
        << "This warning can be ignored during log playback" << std::endl;
    }

    // Make sure eventTime is not negative.
    if (eventTime < common::Time::Zero)
    {
      gzerr << "Time to next sensor update is negative." << std::endl;
      continue;
    }

    boost::mutex::scoped_lock timingLock(g_sensorTimingMutex);

    // Add an event to trigger when the appropriate simulation time has been
    // reached.
    SensorManager::Instance()->simTimeEventHandler->AddRelativeEvent(
        eventTime, &this->runCondition);

    // This if statement helps prevent deadlock on osx during teardown.
    IGN_PROFILE_BEGIN("Sleeping");
    if (!this->stop)
    {
      this->runCondition.wait(timingLock);
    }
    IGN_PROFILE_END();
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
    GZ_ASSERT((*iter) != nullptr, "Sensor is null");
    IGN_PROFILE_BEGIN((*iter)->Name().c_str());
    (*iter)->Update(_force);
    IGN_PROFILE_END();
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
    GZ_ASSERT((*iter) != nullptr, "Sensor is null");

    // We match on the scoped name (model::link::sensor) because multiple
    // sensors with the same leaf name may exist in a world.
    if ((_useLeafName && (*iter)->Name() == _name) ||
        (!_useLeafName && (*iter)->ScopedName() == _name))
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
  GZ_ASSERT(_sensor != nullptr, "Sensor is nullptr when passed to ::AddSensor");

  {
    boost::recursive_mutex::scoped_lock lock(this->mutex);
    this->sensors.push_back(_sensor);
    g_sensorsDirty = true;
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
    GZ_ASSERT((*iter) != nullptr, "Sensor is null");

    if ((*iter)->ScopedName() == _name)
    {
      (*iter)->Fini();
      this->sensors.erase(iter);
      removed = true;
      break;
    }
  }

  g_sensorsDirty = true;

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
    GZ_ASSERT((*iter) != nullptr, "Sensor is null");
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
    GZ_ASSERT((*iter) != nullptr, "Sensor is null");
    (*iter)->Fini();
  }

  g_sensorsDirty = true;

  this->sensors.clear();
}

//////////////////////////////////////////////////
void SensorManager::ImageSensorContainer::Update(bool _force)
{
  // Prerender phase
  event::Events::preRender();

  // Signals end of prerender phase
  event::Events::preRenderEnded();

  // Notify that prerender is over
  this->conditionPrerendered.notify_all();

  // Tell all the cameras to render
  event::Events::render();

  event::Events::postRender();

  // Update the sensors, which will produce data messages.
  SensorContainer::Update(_force);
}

//////////////////////////////////////////////////
bool SensorManager::ImageSensorContainer::WaitForPrerendered(double _timeoutsec)
{
  std::cv_status ret;
  std::mutex mtx;
  std::unique_lock<std::mutex> lck(mtx);
  ret = this->conditionPrerendered.wait_for(lck,
      std::chrono::duration<double>(_timeoutsec));
  return (ret == std::cv_status::no_timeout);
}

//////////////////////////////////////////////////
SensorManager* SensorManager::Instance()
{
#ifndef _WIN32
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  return SingletonT<SensorManager>::Instance();
#ifndef _WIN32
  #pragma GCC diagnostic pop
#endif
}


/////////////////////////////////////////////////
SimTimeEventHandler::SimTimeEventHandler()
{
  using namespace boost::placeholders;
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
    GZ_ASSERT(*iter != nullptr, "SimTimeEvent is null");
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
  GZ_ASSERT(world != nullptr, "World pointer is null");

  // Create the new event.
  SimTimeEvent *event = new SimTimeEvent;
  event->time = world->SimTime() + _time;
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
    GZ_ASSERT(*iter != nullptr, "SimTimeEvent is null");

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
