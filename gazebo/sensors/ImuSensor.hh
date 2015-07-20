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
      protected: virtual bool UpdateImpl(bool _force);

      // Documentation inherited
      protected: virtual void Fini();

      /// \brief Returns the imu message
      /// \return Imu message.
      public: msgs::IMU GetImuMessage() const;

      /// \brief Returns the angular velocity.
      /// \return Angular velocity.
      /// \deprecated See AngularVelocity() function that returns an
      /// ignition::math::Vector3d object.
      public: math::Vector3 GetAngularVelocity() const GAZEBO_DEPRECATED(6.0);

      /// \brief Returns the imu linear acceleration
      /// \return Linear acceleration.
      /// \deprecated See LinearVelocity() function that returns an
      /// ignition::math::Vector3d object.
      public: math::Vector3 GetLinearAcceleration() const
              GAZEBO_DEPRECATED(6.0);

      /// \brief get orientation of the IMU relative to the reference pose
      /// \return returns the orientation quaternion of the IMU relative to
      /// the imu reference pose.
      /// \deprecated See Orientation() function that returns an
      /// ignition::math::Quaterniond object.
      public: math::Quaternion GetOrientation() const GAZEBO_DEPRECATED(6.0);

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
      public: virtual bool IsActive();

      /// \brief Callback when link data is received
      /// \param[in] _msg Message containing link data
      private: void OnLinkData(ConstLinkDataPtr &_msg);

      /// \brief Imu reference pose
      private: ignition::math::Pose3d referencePose;

      /// \brief Save previous imu linear velocity for computing acceleration.
      private: ignition::math::Vector3d lastLinearVel;

      /// \brief Imu linear acceleration
      private: ignition::math::Vector3d linearAcc;

      /// \brief store gravity vector to be added to the imu output.
      private: ignition::math::Vector3d gravity;

      /// \brief Imu data publisher
      private: transport::PublisherPtr pub;

      /// \brief Subscriber to link data published by parent entity
      private: transport::SubscriberPtr linkDataSub;

      /// \brief Parent entity which the IMU is attached to
      private: physics::LinkPtr parentEntity;

      /// \brief Imu message
      private: msgs::IMU imuMsg;

      /// \brief Mutex to protect reads and writes.
      private: mutable boost::mutex mutex;

      /// \brief Buffer for storing link data
      private: boost::shared_ptr<msgs::LinkData const> incomingLinkData[2];

      /// \brief Index for accessing element in the link data array
      private: unsigned int dataIndex;

      /// \brief True if new link data is received
      private: bool dataDirty;

      /// \brief Which noise type we support
      private: enum NoiseModelType
      {
        NONE,
        GAUSSIAN
      };

      /// \brief Noise free imu angular velocity.
      private: ignition::math::Vector3d angularVel;
    };
    /// \}
  }
}
#endif
