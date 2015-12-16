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
#ifndef _GAZEBO_SENSORS_SENSORMANAGER_PRIVATE_HH_
#define _GAZEBO_SENSORS_SENSORMANAGER_PRIVATE_HH_

#include "gazebo/sensors/SensorContainer.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \inernal
    /// \brief A simulation time event
    class SimTimeEvent
    {
      /// \brief The time at which to trigger the condition.
      public: common::Time time;

      /// \brief The condition to notify.
      public: std::condition_variable *condition;
    };

    /// \internal
    /// \brief Monitors simulation time, and notifies conditions when
    /// a specified time has been reached.
    class SimTimeEventHandler
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
                  std::condition_variable *_var);

      /// \brief Called when the world is updated.
      /// \param[in] _info Update timing information.
      public: void OnUpdate(const common::UpdateInfo &_info);

      /// \brief Mutex to mantain thread safety.
      public: std::mutex mutex;

      /// \brief The list of events to handle.
      public: std::list<SimTimeEvent*> events;

      /// \brief Connect to the World::UpdateBegin event.
      public: event::ConnectionPtr updateConnection;
    };

    /// \ineternal
    /// \brief Private date for the sensor manager class
    class SensorManagerPrivate
    {
      /// \brief True if SensorManager::Init has been called
      ///        i.e. SensorManager::sensors are initialized.
      public: bool initialized = false;

      /// \brief True removes all sensors from all sensor containers.
      public: bool removeAllSensors = false;

      /// \brief Mutex used when adding and removing sensors.
      public: mutable std::recursive_mutex mutex;

      /// \brief List of sensors that require initialization.
      public: Sensor_V initSensors;

      /// \brief List of sensors that require initialization.
      public: std::vector<std::string> removeSensors;

      /// \brief A vector of SensorContainer pointers.
      public: typedef std::vector<SensorContainer*> SensorContainer_V;

      /// \brief The sensor manager's vector of sensor containers.
      public: SensorContainer_V sensorContainers;

      /// \brief Pointer to the sim time event handler.
      public: SimTimeEventHandler *simTimeEventHandler;
    };
  }
}
#endif
