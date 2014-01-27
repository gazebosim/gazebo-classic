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

#ifndef _FORCETORQUESENSOR_HH_
#define _FORCETORQUESENSOR_HH_

#include <string>

#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/sensors/Sensor.hh"

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
    class ForceTorqueSensor: public Sensor
    {
      /// \brief Constructor.
      public: ForceTorqueSensor();

      /// \brief Destructor.
      public: virtual ~ForceTorqueSensor();

      // Documentation inherited.
      public: virtual void Load(const std::string &_worldName);

      // Documentation inherited.
      public: virtual void Init();

      // Documentation inherited.
      public: virtual std::string GetTopic() const;

      /// \brief Get the current joint torque.
      /// \return The latested measured torque.
      public: math::Vector3 GetTorque() const;

      /// \brief Get the current joint force.
      /// \return The latested measured force.
      public: math::Vector3 GetForce() const;

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
      protected: virtual void UpdateImpl(bool _force);

      // Documentation inherited.
      protected: virtual void Fini();

      /// \brief Parent joint, from which we get force torque info.
      private: physics::JointPtr parentJoint;

      /// \brief Publishes the wrenchMsg.
      private: transport::PublisherPtr wrenchPub;

      /// \brief Message the store the current force torque info.
      private: msgs::WrenchStamped wrenchMsg;

      /// \brief Mutex to protect the wrench message
      private: boost::mutex mutex;

      /// \brief Update event.
      protected: event::EventT<void(msgs::WrenchStamped)> update;

      /// \brief Get Parent Joint
      /// \return Pointer to the joint containing this sensor
      public: physics::JointPtr GetJoint() const;
    };
    /// \}
  }
}
#endif
