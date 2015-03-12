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
#ifndef _SENSORMANAGER_HH_
#define _SENSORMANAGER_HH_

#include <boost/thread.hpp>
#include <string>
#include <vector>
#include <list>

#include <sdf/sdf.hh>

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/common/SingletonT.hh"
#include "gazebo/common/UpdateInfo.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  /// \ingroup gazebo_sensors
  /// \brief Sensors namespace
  namespace sensors
  {
    /// \cond
    /// \brief A simulation time event
    class GAZEBO_VISIBLE SimTimeEvent
    {
      /// \brief The time at which to trigger the condition.
      public: common::Time time;

      /// \brief The condition to notify.
      public: boost::condition_variable *condition;
    };

    /// \brief Monitors simulation time, and notifies conditions when
    /// a specified time has been reached.
    class GAZEBO_VISIBLE SimTimeEventHandler
    {
      /// \brief Constructor
      public: SimTimeEventHandler();

      /// \brief Destructor
      public: virtual ~SimTimeEventHandler();

      /// \brief Add a new event to the handler.
      /// \param[in] _time Time of the new event. The current sim time will
      /// be add to this time.
      /// \param[in] _var Condition to notify when the time has been
      /// reached.
      public: void AddRelativeEvent(const common::Time &_time,
                  boost::condition_variable *_var);

      /// \brief Called when the world is updated.
      /// \param[in] _info Update timing information.
      private: void OnUpdate(const common::UpdateInfo &_info);

      /// \brief Mutex to mantain thread safety.
      private: boost::mutex mutex;

      /// \brief The list of events to handle.
      private: std::list<SimTimeEvent*> events;

      /// \brief Connect to the World::UpdateBegin event.
      private: event::ConnectionPtr updateConnection;
    };
    /// \endcond

    /// \addtogroup gazebo_sensors
    /// \{
    /// \class SensorManager SensorManager.hh sensors/sensors.hh
    /// \brief Class to manage and update all sensors
    class GAZEBO_VISIBLE SensorManager : public SingletonT<SensorManager>
    {
      /// \brief This is a singletone class. Use SensorManager::Instance()
      /// to get a pointer to this class.
      private: SensorManager();

      /// \brief Destructor
      private: virtual ~SensorManager();

      /// \brief Update all the sensors
      ///
      /// Checks to see if any sensor need to be initialized first,
      /// then updates all sensors once.
      /// \param[in] _force True force update, false if not
      public: void Update(bool _force = false);

      /// \brief Init all the sensors
      public: void Init();

      /// \brief Run sensor updates in separate threads.
      /// This will only run non-image based sensor updates.
      public: void RunThreads();

      /// \brief Stop the run thread
      public: void Stop();

      /// \brief Finalize all the sensors
      public: void Fini();

      /// \brief Get all the sensor types
      /// \param[out] All the sensor types.
      public: void GetSensorTypes(std::vector<std::string> &_types) const;

      /// \brief Add a sensor from an SDF element. This function will also Load
      /// and Init the sensor.
      /// \param[in] _elem The SDF element that describes the sensor
      /// \param[in] _worldName Name of the world in which to create the sensor
      /// \param[in] _parentName The name of the parent link which the sensor is
      /// attached to.
      /// \return The name of the sensor
      public: std::string CreateSensor(sdf::ElementPtr _elem,
                                       const std::string &_worldName,
                                       const std::string &_parentName,
                                       uint32_t _parentId);

      /// \brief Get a sensor
      /// \param[in] _name The name of a sensor to find.
      /// \return A pointer to the sensor. NULL if not found.
      public: SensorPtr GetSensor(const std::string &_name) const;

      /// \brief Get all the sensors.
      /// \return Vector of all the sensors.
      public: Sensor_V GetSensors() const;

      /// \brief Remove a sensor
      /// \param[in] _name The name of the sensor to remove.
      public: void RemoveSensor(const std::string &_name);

      /// \brief Remove all sensors
      public: void RemoveSensors();

      /// \brief True if SensorManager::initSensors queue is empty
      /// i.e. all sensors managed by SensorManager have been initialized
      public: bool SensorsInitialized();

      /// \brief Reset last update times in all sensors.
      public: void ResetLastUpdateTimes();

      /// \brief Add a new sensor to a sensor container.
      /// \param[in] _sensor Pointer to a sensor to add.
      private: void AddSensor(SensorPtr _sensor);

      /// \cond
      /// \brief A container for sensors of a specific type. This is used to
      /// separate sensors which rely on the rendering engine from those
      /// that do not.
      /// This is a private embedded class because only the SensorManager
      /// should have access to SensorContainers.
      private: class SensorContainer
               {
                 /// \brief Constructor
                 public: SensorContainer();

                 /// \brief Destructor
                 public: virtual ~SensorContainer();

                 /// \brief Initialize all sensors in this container.
                 public: void Init();

                 /// \brief Finalize all sensors in this container.
                 public: void Fini();

                 /// \brief Run the sensor updates in a separate thread.
                 public: void Run();

                 /// \brief Stop the run thread.
                 public: void Stop();

                 /// \brief Update the sensors.
                 /// \param[in] _force True to force the sensors to update,
                 /// even if they are not active.
                 public: virtual void Update(bool _force = false);

                 /// \brief Add a new sensor to this container.
                 /// \param[in] _sensor Pointer to a sensor to add.
                 public: void AddSensor(SensorPtr _sensor);

                 /// \brief Get a sensor by name.
                 /// \param[in] _useLeafName False indicates that _name
                 /// should be compared against the scoped name of a sensor.
                 /// \return Pointer to the matching sensor. NULL if no
                 /// sensor is found.
                 public: SensorPtr GetSensor(const std::string &_name,
                                             bool _useLeafName = false) const;

                 /// \brief Remove a sensor by name.
                 /// \param[in] _name Name of the sensor to remove, which
                 /// must be a scoped name.
                 public: bool RemoveSensor(const std::string &_name);

                 /// \brief Remove all sensors.
                 public: void RemoveSensors();

                 /// \brief Reset last update times in all sensors.
                 public: void ResetLastUpdateTimes();

                 /// \brief A loop to update the sensor. Used by the
                 /// runThread.
                 private: void RunLoop();

                 /// \brief The set of sensors to maintain.
                 public: Sensor_V sensors;

                 /// \brief Flag to inidicate when to stop the runThread.
                 private: bool stop;

                 /// \brief Flag to indicate that the sensors have been
                 /// initialized.
                 private: bool initialized;

                 /// \brief A thread to update the sensors.
                 private: boost::thread *runThread;

                 /// \brief A mutex to manage access to the sensors vector.
                 private: mutable boost::recursive_mutex mutex;

                 /// \brief Condition used to block the RunLoop if no
                 /// sensors are present.
                 private: boost::condition_variable runCondition;
               };
      /// \endcond

      /// \cond
      /// \brief Image based sensors need a special update. So we subclass
      /// the SensorContainer.
      private: class ImageSensorContainer : public SensorContainer
               {
                 /// \brief The special update for image based sensors.
                 /// \param[in] _force True to force the sensors to update,
                 /// even if they are not active.
                 public: virtual void Update(bool _force = false);
               };
      /// \endcond

      /// \brief True if SensorManager::Init has been called
      ///        i.e. SensorManager::sensors are initialized.
      private: bool initialized;

      /// \brief True removes all sensors from all sensor containers.
      private: bool removeAllSensors;

      /// \brief Mutex used when adding and removing sensors.
      private: mutable boost::recursive_mutex mutex;

      /// \brief List of sensors that require initialization.
      private: Sensor_V initSensors;

      /// \brief List of sensors that require initialization.
      private: std::vector<std::string> removeSensors;

      /// \brief A vector of SensorContainer pointers.
      private: typedef std::vector<SensorContainer*> SensorContainer_V;

      /// \brief The sensor manager's vector of sensor containers.
      private: SensorContainer_V sensorContainers;

      /// \brief This is a singleton class.
      private: friend class SingletonT<SensorManager>;

      /// \brief Allow access to sensorTimeMutex member var.
      private: friend class SensorContainer;

      /// \brief Pointer to the sim time event handler.
      private: SimTimeEventHandler *simTimeEventHandler;
    };
    /// \}
  }
}
#endif
