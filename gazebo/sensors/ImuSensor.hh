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
#ifndef _GAZEBO_IMUSENSOR_HH_
#define _GAZEBO_IMUSENSOR_HH_

#include <vector>
#include <string>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/sensors/Sensor.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace sensors
  {
    // Forward declare private data class.
    class ImuSensorPrivate;

    /// \addtogroup gazebo_sensors
    /// \{

    /// \class ImuSensor ImuSensor.hh sensors/sensors.hh
    /// \brief An IMU sensor.
    class GAZEBO_VISIBLE ImuSensor: public Sensor
    {
      /// \brief Constructor.
      public: ImuSensor();

      /// \brief Destructor.
      public: virtual ~ImuSensor();

      // Documentation inherited.
      protected: void Load(const std::string &_worldName, sdf::ElementPtr _sdf);

      // Documentation inherited.
      protected: virtual void Load(const std::string &_worldName);

      /// \brief Initialize the IMU.
      public: virtual void Init();

      // Documentation inherited
      protected: virtual bool UpdateImpl(const bool _force);

      // Documentation inherited
      protected: virtual void Fini();

      /// \brief Returns the imu message
      /// \return Imu message.
      /// \deprecated See ImuMessage()
      public: msgs::IMU GetImuMessage() const GAZEBO_DEPRECATED(7.0);

      /// \brief Returns the imu message
      /// \return Imu message.
      public: msgs::IMU ImuMessage() const;

      /// \brief Returns the angular velocity.
      /// \param[in] _noiseFree True if the returned measurement should
      /// not use noise.
      /// \return Angular velocity.
      public: ignition::math::Vector3d AngularVelocity(
                  const bool _noiseFree = false) const;

      /// \brief Returns the imu linear acceleration
      /// \param[in] _noiseFree True if the returned measurement should
      /// not use noise.
      /// \return Linear acceleration.
      public: ignition::math::Vector3d LinearAcceleration(
                  const bool _noiseFree = false) const;

      /// \brief get orientation of the IMU relative to the reference pose
      /// \return returns the orientation quaternion of the IMU relative to
      /// the imu reference pose.
      public: ignition::math::Quaterniond Orientation() const;

      /// \brief Sets the current pose as the IMU reference pose
      public: void SetReferencePose();

      // Documentation inherited.
      public: virtual bool IsActive() const;

      /// \brief Callback when link data is received
      /// \param[in] _msg Message containing link data
      private: void OnLinkData(ConstLinkDataPtr &_msg);

      /// \internal
      /// \brief Private data pointer.
      private: std::shared_ptr<ImuSensorPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
