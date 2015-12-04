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
#ifndef _GAZEBO_SENSOR_HH_
#define _GAZEBO_SENSOR_HH_

#include <boost/enable_shared_from_this.hpp>
#include <boost/thread/mutex.hpp>
#include <vector>
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
    // Forward declare protected data
    class SensorProtected;

    // Forward declare private data
    class SensorPrivate;

    /// \addtogroup gazebo_sensors
    /// \{

    /// \class Sensor Sensor.hh sensors/sensors.hh
    /// \brief Base class for sensors
    class GAZEBO_VISIBLE Sensor : public boost::enable_shared_from_this<Sensor>
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
      /// \deprecated See ParentName() function.
      public: std::string GetParentName() const
              GAZEBO_DEPRECATED(7.0);

      /// \brief Returns the name of the sensor parent.  The parent name is
      ///        set by Sensor::SetParent.
      /// \return Name of Parent.
      public: std::string ParentName() const;

      /// \brief Update the sensor.
      /// \param[in] _force True to force update, false otherwise.
      public: void Update(const bool _force);

      /// \brief Get the update rate of the sensor.
      /// \return _hz update rate of sensor.  Returns 0 if unthrottled.
      /// \deprecated See UpdateRate() function
      public: double GetUpdateRate() GAZEBO_DEPRECATED(7.0);

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
      /// \deprecated See Name() function.
      public: std::string GetName() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get name.
      /// \return Name of sensor.
      public: std::string Name() const;

      /// \brief Get fully scoped name of the sensor.
      /// \return world_name::model_name::link_name::sensor_name.
      /// \deprecated See ScopedName() function
      public: std::string GetScopedName() const GAZEBO_DEPRECATED(7.0);

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
      /// \deprecated See Type() function.
      public: std::string GetType() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get sensor type.
      /// \return Type of sensor.
      public: std::string Type() const;

      /// \brief Return last update time.
      /// \return Time of last update.
      /// \deprecated See LastUpdateTime() function
      public: common::Time GetLastUpdateTime() GAZEBO_DEPRECATED(7.0);

      /// \brief Return last update time.
      /// \return Time of last update.
      public: common::Time LastUpdateTime() const;

      /// \brief Return last measurement time.
      /// \return Time of last measurement.
      /// \deprecated See LastMeasurementTime() function.
      public: common::Time GetLastMeasurementTime() GAZEBO_DEPRECATED(7.0);

      /// \brief Return last measurement time.
      /// \return Time of last measurement.
      /// \deprecated See LastMeasurementTime() function.
      public: common::Time LastMeasurementTime() const;

      /// \brief Return true if user requests the sensor to be visualized
      ///        via tag:  <visualize>true</visualize> in SDF.
      /// \return True if visualized, false if not.
      /// \deprecated See Visualize() function
      public: bool GetVisualize() const GAZEBO_DEPRECATED(7.0);

      /// \brief Return true if user requests the sensor to be visualized
      ///        via tag:  <visualize>true</visualize> in SDF.
      /// \return True if visualized, false if not.
      /// \deprecated See Visualize() function
      public: bool Visualize() const;

      /// \brief Returns the topic name as set in SDF.
      /// \return Topic name.
      /// \deprecated See Topic() function.
      public: virtual std::string GetTopic() const GAZEBO_DEPRECATED(7.0);

      /// \brief Returns the topic name as set in SDF.
      /// \return Topic name.
      public: virtual std::string Topic() const;

      /// \brief fills a msgs::Sensor message.
      /// \param[out] _msg Message to fill.
      public: void FillMsg(msgs::Sensor &_msg);

      /// \brief Returns the name of the world the sensor is in.
      /// \return Name of the world.
      /// \deprecated See WorldName() function.
      public: std::string GetWorldName() const GAZEBO_DEPRECATED(7.0);

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
      public: void DisconnectUpdated(event::ConnectionPtr &_c);

      /// \brief Get the category of the sensor.
      /// \return The category of the sensor.
      /// \sa SensorCategory
      /// \deprecated See Category() function.
      public: SensorCategory GetCategory() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the category of the sensor.
      /// \return The category of the sensor.
      /// \sa SensorCategory
      public: SensorCategory Category() const;

      /// \brief Reset the lastUpdateTime to zero.
      public: void ResetLastUpdateTime();

      /// \brief Get the sensor's ID.
      /// \return The sensor's ID.
      /// \deprecated See Id() function
      public: uint32_t GetId() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the sensor's ID.
      /// \return The sensor's ID.
      /// \deprecated See Id() function
      public: uint32_t Id() const;

      /// \brief Get the sensor's parent's ID.
      /// \return The sensor's parent's ID.
      /// \deprecated See ParentId() function
      public: uint32_t GetParentId() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the sensor's parent's ID.
      /// \return The sensor's parent's ID.
      public: uint32_t ParentId() const;

      /// \brief Get the sensor's noise model for a specified noise type.
      /// \param[in] _type Index of the noise type. Refer to
      /// SensorNoiseType enumeration for possible indices
      /// \return The sensor's noise model for the given noise type
      /// \deprecated See Noise(const SensorNoiseType _type) function
      public: NoisePtr GetNoise(const SensorNoiseType _type) const
              GAZEBO_DEPRECATED(7.0);

      /// \brief Get the sensor's noise model for a specified noise type.
      /// \param[in] _type Index of the noise type. Refer to
      /// SensorNoiseType enumeration for possible indices
      /// \return The sensor's noise model for the given noise type
      /// \deprecated See Noise(const SensorNoiseType _type) function
      public: NoisePtr Noise(const SensorNoiseType _type) const;

      /// \internal
      /// \brief Constructor used by inherited classes
      /// \param[in] _dataPtr Pointer to private data.
      /// \param[in] _cat Category of the sensor
      protected: Sensor(SensorProtected &_dataPtr, SensorCategory _cat);

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

      /// \brief Shared construction code. This should only be called from
      /// a constructor.
      /// \param[in] _cat Category of the sensor
      private: void ConstructorHelper(SensorCategory _cat);

      /// \internal
      /// \brief Data pointer for protected data
      protected: std::shared_ptr<SensorProtected> dPtr;

      /// \internal
      /// \brief Data pointer for private data
      private: std::unique_ptr<SensorPrivate> pdPtr;
    };
    /// \}
  }
}
#endif
