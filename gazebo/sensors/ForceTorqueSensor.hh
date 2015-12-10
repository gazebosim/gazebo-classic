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
#ifndef _GAZEBO_FORCETORQUESENSOR_HH_
#define _GAZEBO_FORCETORQUESENSOR_HH_

#include <string>

#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/sensors/Sensor.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  /// \ingroup gazebo_sensors
  /// \brief Sensors namespace
  namespace sensors
  {
    /// \addtogroup gazebo_sensors
    /// \{

    /// \class ForceTorqueSensor ForceTorqueSensor.hh sensors/sensors.hh
    /// \brief Sensor for measure force and torque on a joint.
    class GAZEBO_VISIBLE ForceTorqueSensor: public Sensor
    {
      /// \brief Constructor.
      public: ForceTorqueSensor();

      /// \brief Destructor.
      public: virtual ~ForceTorqueSensor();

      // Documentation inherited.
      protected: void Load(const std::string &_worldName, sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: virtual void Load(const std::string &_worldName);

      // Documentation inherited.
      public: virtual void Init();

      // Documentation inherited.
      public: virtual std::string GetTopic() const;

      /// \brief Get the current joint torque.
      /// \return The latest measured torque.
      public: ignition::math::Vector3d Torque() const;

      /// \brief Get the current joint force.
      /// \return The latested measured force.
      public: ignition::math::Vector3d Force() const;

      /// \brief Get Parent Joint
      /// \return Pointer to the joint containing this sensor
      public: physics::JointPtr GetJoint() const;

      // Documentation inherited.
      public: virtual bool IsActive();

      /// \brief Connect a to the  update signal.
      /// \param[in] _subscriber Callback function.
      /// \return The connection, which must be kept in scope.
      public: template<typename T>
              event::ConnectionPtr ConnectUpdate(T _subscriber)
              {return update.Connect(_subscriber);}

      /// \brief Disconnect from the update signal.
      /// \param[in] _conn Connection to remove.
      public: void DisconnectUpdate(event::ConnectionPtr &_conn)
              {update.Disconnect(_conn);}

      // Documentation inherited.
      protected: virtual bool UpdateImpl(bool _force);

      // Documentation inherited.
      protected: virtual void Fini();

      /// \brief Update event.
      protected: event::EventT<void(msgs::WrenchStamped)> update;

      /// \brief Parent joint, from which we get force torque info.
      private: physics::JointPtr parentJoint;

      /// \brief Publishes the wrenchMsg.
      private: transport::PublisherPtr wrenchPub;

      /// \brief Message the store the current force torque info.
      private: msgs::WrenchStamped wrenchMsg;

      /// \brief Mutex to protect the wrench message
      private: boost::mutex mutex;

      /// \brief Which orientation we support for returning sensor measure
      private: enum MeasureFrame
      {
        PARENT_LINK,
        CHILD_LINK,
        SENSOR
      };

      /// \brief Frame in which we return the measured force torque info.
      private: MeasureFrame measureFrame;

      /// \brief Direction of the measure
      ///        True if the measured force torque is the one applied
      ///        by the parent on the child, false otherwise
      private: bool parentToChild;

      /// \brief Rotation matrix than transforms a vector expressed in child
      ///        orientation in a vector expressed in joint orientation.
      ///        Necessary is the measure is specified in joint frame.
      private: ignition::math::Matrix3d rotationSensorChild;
    };
    /// \}
  }
}
#endif
