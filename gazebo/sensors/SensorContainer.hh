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
#ifndef _GAZEBO_SENSORS_SENSORCONTAINER_HH_
#define _GAZEBO_SENSORS_SENSORCONTAINER_HH_

namespace gazebo
{
  namespace sensors
  {
    /// \internal
    /// \brief A container for sensors of a specific type. This is used to
    /// separate sensors which rely on the rendering engine from those
    /// that do not.
    /// This is a private embedded class because only the SensorManager
    /// should have access to SensorContainers.
    class SensorContainer
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
      public: void RunLoop();

      /// \brief The set of sensors to maintain.
      public: Sensor_V sensors;

      /// \brief Flag to inidicate when to stop the runThread.
      public: bool stop;

      /// \brief Flag to indicate that the sensors have been
      /// initialized.
      public: bool initialized;

      /// \brief A thread to update the sensors.
      public: std::thread *runThread;

      /// \brief A mutex to manage access to the sensors vector.
      public: mutable std::recursive_mutex mutex;

      /// \brief Condition used to block the RunLoop if no
      /// sensors are present.
      public: std::condition_variable runCondition;
    };

    /// \internal
    /// \brief Image based sensors need a special update. So we subclass
    /// the SensorContainer.
    class ImageSensorContainer : public SensorContainer
    {
      /// \brief The special update for image based sensors.
      /// \param[in] _force True to force the sensors to update,
      /// even if they are not active.
      public: virtual void Update(bool _force = false);
    };
  }
}
#endif
