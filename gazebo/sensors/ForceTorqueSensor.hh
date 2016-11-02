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
#ifndef _GAZEBO_SENSORS_FORCETORQUESENSOR_HH_
#define _GAZEBO_SENSORS_FORCETORQUESENSOR_HH_

#include <functional>
#include <memory>
#include <string>

#include <ignition/math/Vector3.hh>

#include "gazebo/sensors/Sensor.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  /// \ingroup gazebo_sensors
  /// \brief Sensors namespace
  namespace sensors
  {
    // Forward declare private data class.
    class ForceTorqueSensorPrivate;

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
      public: virtual std::string Topic() const;

      /// \brief Get the current joint torque.
      /// \return The latest measured torque.
      public: ignition::math::Vector3d Torque() const;

      /// \brief Get the current joint force.
      /// \return The latested measured force.
      public: ignition::math::Vector3d Force() const;

      /// \brief Get Parent Joint
      /// \return Pointer to the joint containing this sensor
      public: physics::JointPtr Joint() const;

      // Documentation inherited.
      public: virtual bool IsActive() const;

      /// \brief Connect a to the  update signal.
      /// \param[in] _subscriber Callback function.
      /// \return The connection, which must be kept in scope.
      /// \deprecated See ConnectUpdate that accepts a std::function
      /// parameter.
      public: event::ConnectionPtr ConnectUpdate(
                  std::function<void (msgs::WrenchStamped)> _subscriber);

      /// \brief Disconnect from the update signal.
      /// \param[in] _conn Connection to remove.
      /// \deprecated Use event::~Connection to disconnect
      public: void DisconnectUpdate(event::ConnectionPtr &_conn)
              GAZEBO_DEPRECATED(8.0);

      // Documentation inherited.
      protected: virtual bool UpdateImpl(const bool _force);

      // Documentation inherited.
      protected: virtual void Fini();

      /// \internal
      /// \brief Private data pointer
      private: std::unique_ptr<ForceTorqueSensorPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
