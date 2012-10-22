/*
 * Copyright 2011 Nate Koenig
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
/* Desc: Base class for all sensors
 * Author: Nathan Koenig
 * Date: 25 May 2007
 */

#ifndef SENSOR_HH
#define SENSOR_HH

#include <boost/enable_shared_from_this.hpp>
#include <vector>
#include <string>

#include "sdf/sdf.hh"

#include "physics/PhysicsTypes.hh"

#include "msgs/msgs.hh"
#include "common/Events.hh"
#include "common/Time.hh"
#include "math/Pose.hh"
#include "transport/TransportTypes.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \addtogroup gazebo_sensors
    /// \{

    /// \class Sensor Sensor.hh sensors/Sensor.hh
    /// \brief Base class for sensors
    class Sensor : public boost::enable_shared_from_this<Sensor>
    {
      /// \brief  Constructor
      public: Sensor();

      /// \brief  Destructor
      public: virtual ~Sensor();

      /// \brief Load the sensor with SDF parameters
      /// \param _sdf SDF Sensor parameteres
      public: virtual void Load(const std::string &_worldName,
                                sdf::ElementPtr _sdf);

      /// \brief Load the sensor with default parameters
      public: virtual void Load(const std::string &_worldName);

      /// \brief  Initialize the sensor
      public: virtual void Init();

      /// \brief Set the parent of the sensor
      public: virtual void SetParent(const std::string &_name);

      /// \brief Returns the name of the sensor parent.  The parent name is
      ///        set by Sensor::SetParent.
      public: std::string GetParentName() const;

      /// \brief  Update the sensor.
      public: void Update(bool _force);

      /// \brief  This gets overwritten by derived sensor types.
      ///         This function is called during Sensor::Update.
      ///         And in turn, Sensor::Update is called by
      ///         SensorManager::Update
      protected: virtual void UpdateImpl(bool /*_force*/) {}

      /// \brief Set the update rate of the sensor
      public: void SetUpdateRate(double _hz);

      /// \brief  Finalize the sensor
      public: virtual void Fini();

      /// \brief Get the sensor's name
      /// \return The unscoped name of the sensor.
      /// \sa Sensor::GetScopedName
      public: std::string GetName() const;

      /// \brief Get fully scoped name of the sensor
      /// \return world_name::parent_name::sensor_name
      public: std::string GetScopedName() const;

      /// \brief Get the current pose
      public: virtual math::Pose GetPose() const;

      /// \brief Set whether the sensor is active or not
      public: virtual void SetActive(bool value);

      /// \brief Returns true if sensor generation is active.
      public: bool IsActive();

      /// \brief Get sensor type
      public: std::string GetType() const;

      /// \brief return last update time
      public: common::Time GetLastUpdateTime();

      /// \brief return true if user requests the sensor to be visualized
      ///        via tag:  <visualize>true</visualize> in SDF
      public: bool GetVisualize() const;

      /// \brief Returns the topic name as set in SDF.
      public: virtual std::string GetTopic() const;

      /// \brief fills a msgs::Sensor message.
      public: void FillMsg(msgs::Sensor &_msg);

      /// \brief Returns the name of the world the sensor is in.
      public: std::string GetWorldName() const;

      /// \brief Load a plugin for this sensor
      /// \param _sdf SDF parameters
      private: void LoadPlugin(sdf::ElementPtr _sdf);

      /// \brief Callback when a world control message is received.
      /// \param[in] _data The world control message.
      private: void OnControl(ConstWorldControlPtr &_data);

      /// \brief True if active
      protected: bool active;
      protected: sdf::ElementPtr sdf;
      protected: math::Pose pose;
      protected: std::vector<event::ConnectionPtr> connections;
      protected: transport::NodePtr node;
      protected: transport::SubscriberPtr poseSub;
      private: transport::SubscriberPtr controlSub;
      private: transport::PublisherPtr sensorPub;

      protected: std::string parentName;
      protected: std::vector<SensorPluginPtr> plugins;

      protected: gazebo::physics::WorldPtr world;
      protected: common::Time updatePeriod;
      protected: common::Time lastUpdateTime;
    };
    /// \}
  }
}
#endif
