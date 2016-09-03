/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_SENSORS_SENSOR_HH_
#define _GAZEBO_SENSORS_SENSOR_HH_

#include <vector>
#include <memory>
#include <map>
#include <string>

#include <sdf/sdf.hh>
#include <ignition/math/Pose3.hh>

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/sensors/SensorTypes.hh"

#include "gazebo/msgs/msgs.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace sensors
  {
    // Forward declare private data
    class SensorPrivate;

    /// \addtogroup gazebo_sensors
    /// \{

    /// \class Sensor Sensor.hh sensors/sensors.hh
    /// \brief Base class for sensors
    class GAZEBO_VISIBLE Sensor : public std::enable_shared_from_this<Sensor>
    {
      /// \brief Constructor.
      /// \param[in] _cat Category of the sensor
      public: explicit Sensor(SensorCategory _cat);

      /// \brief Destructor.
      public: virtual ~Sensor();

      /// \brief Load the sensor with SDF parameters.
      /// \param[in] _sdf SDF Sensor parameters.
      /// \param[in] _worldName Name of world to load from.
      public: virtual void Load(const std::string &_worldName,
                                sdf::ElementPtr _sdf);

      /// \brief Load the sensor with default parameters.
      /// \param[in] _worldName Name of world to load from.
      public: virtual void Load(const std::string &_worldName);

      /// \brief Initialize the sensor.
      public: virtual void Init();

      /// \brief Set the sensor's parent.
      /// \param[in] _name The sensor's parent's name.
      /// \param[in] _id The sensor's parent's ID.
      public: void SetParent(const std::string &_name, const uint32_t _id);

      /// \brief Returns the name of the sensor parent.  The parent name is
      ///        set by Sensor::SetParent.
      /// \return Name of Parent.
      public: std::string ParentName() const;

      /// \brief Update the sensor.
      /// \param[in] _force True to force update, false otherwise.
      public: void Update(const bool _force);

      /// \brief Get the update rate of the sensor.
      /// \return _hz update rate of sensor.  Returns 0 if unthrottled.
      public: double UpdateRate() const;

      /// \brief Set the update rate of the sensor.
      /// \param[in] _hz update rate of sensor.
      public: void SetUpdateRate(const double _hz);

      /// \brief Finalize the sensor.
      public: virtual void Fini();

      /// \brief Get name.
      /// \return Name of sensor.
      public: std::string Name() const;

      /// \brief Get fully scoped name of the sensor.
      /// \return world_name::model_name::link_name::sensor_name.
      public: std::string ScopedName() const;

      /// \brief Get the current pose.
      /// \return Current pose of the sensor.
      /// \sa SetPose()
      public: virtual ignition::math::Pose3d Pose() const;

      /// \brief Set the current pose.
      /// \param[in] _pose New pose of the sensor.
      /// \sa Pose()
      public: virtual void SetPose(const ignition::math::Pose3d &_pose);

      /// \brief Set whether the sensor is active or not.
      /// \param[in] _value True if active, false if not.
      public: virtual void SetActive(const bool _value);

      /// \brief Returns true if sensor generation is active.
      /// \return True if active, false if not.
      public: virtual bool IsActive() const;

      /// \brief Get sensor type.
      /// \return Type of sensor.
      public: std::string Type() const;

      /// \brief Return last update time.
      /// \return Time of last update.
      public: common::Time LastUpdateTime() const;

      /// \brief Return last measurement time.
      /// \return Time of last measurement.
      /// \deprecated See LastMeasurementTime() function.
      public: common::Time LastMeasurementTime() const;

      /// \brief Return true if user requests the sensor to be visualized
      ///        via tag:  <visualize>true</visualize> in SDF.
      /// \return True if visualized, false if not.
      public: bool Visualize() const;

      /// \brief Returns the topic name as set in SDF.
      /// \return Topic name.
      public: virtual std::string Topic() const;

      /// \brief fills a msgs::Sensor message.
      /// \param[out] _msg Message to fill.
      public: void FillMsg(msgs::Sensor &_msg);

      /// \brief Returns the name of the world the sensor is in.
      /// \return Name of the world.
      public: std::string WorldName() const;

      /// \brief Connect a signal that is triggered when the sensor is
      /// updated.
      /// \param[in] _subscriber Callback that receives the signal.
      /// \return A pointer to the connection. This must be kept in scope.
      /// \sa Sensor::DisconnectUpdated
      public: event::ConnectionPtr ConnectUpdated(
                  std::function<void()> _subscriber);

      /// \brief Disconnect from a the updated signal.
      /// \param[in] _c The connection to disconnect
      /// \sa Sensor::ConnectUpdated
      /// \deprecated Use event::~Connection to disconnect
      public: void DisconnectUpdated(event::ConnectionPtr &_c)
              GAZEBO_DEPRECATED(8.0);

      /// \brief Get the category of the sensor.
      /// \return The category of the sensor.
      /// \sa SensorCategory
      public: SensorCategory Category() const;

      /// \brief Reset the lastUpdateTime to zero.
      public: void ResetLastUpdateTime();

      /// \brief Get the sensor's ID.
      /// \return The sensor's ID.
      public: uint32_t Id() const;

      /// \brief Get the sensor's parent's ID.
      /// \return The sensor's parent's ID.
      public: uint32_t ParentId() const;

      /// \brief Get the sensor's noise model for a specified noise type.
      /// \param[in] _type Index of the noise type. Refer to
      /// SensorNoiseType enumeration for possible indices
      /// \return The sensor's noise model for the given noise type
      public: NoisePtr Noise(const SensorNoiseType _type) const;

      /// \brief This gets overwritten by derived sensor types.
      ///        This function is called during Sensor::Update.
      ///        And in turn, Sensor::Update is called by
      ///        SensorManager::Update
      /// \param[in] _force True if update is forced, false if not
      /// \return True if the sensor was updated.
      protected: virtual bool UpdateImpl(const bool /*_force*/) {return false;}

      /// \brief Return true if the sensor needs to be updated.
      /// \return True when sensor should be updated.
      protected: bool NeedsUpdate();

      /// \brief Load a plugin for this sensor.
      /// \param[in] _sdf SDF parameters.
      private: void LoadPlugin(sdf::ElementPtr _sdf);

      /// \brief True if sensor generation is active.
      protected: bool active;

      /// \brief Pointer the the SDF element for the sensor.
      protected: sdf::ElementPtr sdf;

      /// \brief Pose of the sensor.
      protected: ignition::math::Pose3d pose;

      /// \brief All event connections.
      protected: std::vector<event::ConnectionPtr> connections;

      /// \brief Node for communication.
      protected: transport::NodePtr node;

      /// \brief Name of the parent.
      protected: std::string parentName;

      /// \brief The sensor's parent ID.
      protected: uint32_t parentId;

      /// \brief All the plugins for the sensor.
      protected: std::vector<SensorPluginPtr> plugins;

      /// \brief Pointer to the world.
      protected: gazebo::physics::WorldPtr world;

      /// \brief Pointer to the Scene
      protected: gazebo::rendering::ScenePtr scene;

      /// \brief Desired time between updates, set indirectly by
      ///        Sensor::SetUpdateRate.
      protected: common::Time updatePeriod;

      /// \brief Time of the last update.
      protected: common::Time lastUpdateTime;

      /// \brief Stores last time that a sensor measurement was generated;
      ///        this value must be updated within each sensor's UpdateImpl
      protected: common::Time lastMeasurementTime;

      /// \brief Noise added to sensor data
      protected: std::map<SensorNoiseType, NoisePtr> noises;

      /// \internal
      /// \brief Data pointer for private data
      private: std::unique_ptr<SensorPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
