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
#ifndef GAZEBO_SENSORS_IMUSENSOR_HH_
#define GAZEBO_SENSORS_IMUSENSOR_HH_

#include <memory>
#include <string>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

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
      public: msgs::IMU ImuMessage() const;

      /// \brief Returns the angular velocity in the IMU sensor local frame.
      /// \param[in] _noiseFree True if the returned measurement should
      /// not use noise.
      /// \return Angular velocity.
      public: ignition::math::Vector3d AngularVelocity(
                  const bool _noiseFree = false) const;

      /// \brief Returns the imu linear acceleration in the IMU sensor
      /// local frame
      /// \param[in] _noiseFree True if the returned measurement should
      /// not use noise.
      /// \return Linear acceleration.
      public: ignition::math::Vector3d LinearAcceleration(
                  const bool _noiseFree = false) const;

      /// \brief get orientation of the IMU relative to a reference pose
      /// Initially, the reference pose is the boot up pose of the IMU,
      /// but user can call either SetReferencePose to define current
      /// pose as the reference frame, or call SetWorldToReferencePose
      /// to define transform from world frame to reference frame.
      /// \return returns the orientation quaternion of the IMU relative to
      /// the imu reference pose.
      public: ignition::math::Quaterniond Orientation() const;

      /// \brief Sets the current IMU pose as the reference NED pose,
      /// i.e. X axis of the IMU is aligned with North,
      ///      Y axis of the IMU is aligned with East,
      ///      Z axis of the IMU is aligned with Downward (gravity) direction.
      public: void SetReferencePose();

      // Documentation inherited.
      public: virtual bool IsActive() const;

      /// \brief Sets the transform from world frame to IMU's reference frame.
      /// For example, if this IMU works with respect to NED frame, then
      /// call this function with the transform that transforms world frame
      /// to NED frame. Subsequently, ImuSensor::Orientation will return
      /// identity transform if the IMU is aligned with the NED frame.
      /// This call replaces SetReferencePose.
      /// \param _pose rotation from world frame to imu reference frame,
      /// tranlation part of _pose param is ignored.
      /// \deprecated See SetWorldToReferenceOrientation(Quaterniond)
      public: void SetWorldToReferencePose(
        const ignition::math::Pose3d &_pose = ignition::math::Pose3d())
        GAZEBO_DEPRECATED(8.0);

      /// \brief Sets the rotation transform from world frame to IMU's
      /// reference frame.
      /// For example, if this IMU works with respect to NED frame, then
      /// call this function with the transform that transforms world frame
      /// to NED frame. Subsequently, ImuSensor::Orientation will return
      /// identity transform if the IMU is aligned with the NED frame.
      /// This call replaces SetWorldToReferencePose.
      /// \param _orientation rotation from world frame to imu reference frame.
      public: void SetWorldToReferenceOrientation(
        const ignition::math::Quaterniond &_orientation);

      /// \brief Callback when link data is received
      /// \param[in] _msg Message containing link data
      private: void OnLinkData(ConstLinkDataPtr &_msg);

      /// \internal
      /// \brief Private data pointer.
      private: std::unique_ptr<ImuSensorPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
