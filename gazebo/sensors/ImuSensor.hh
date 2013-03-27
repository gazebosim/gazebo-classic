/*
 * Copyright 2012 Open Source Robotics Foundation
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

#ifndef _IMUSENSOR_HH_
#define _IMUSENSOR_HH_

#include <vector>
#include <string>

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/sensors/Sensor.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \addtogroup gazebo_sensors
    /// \{

    /// \class ImuSensor ImuSensor.hh sensors/sensors.hh
    /// \brief An IMU sensor.
    class ImuSensor: public Sensor
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
      protected: virtual void Init();

      // Documentation inherited
      protected: virtual void UpdateImpl(bool _force);

      // Documentation inherited
      protected: virtual void Fini();

      /// \brief Callback when link data is received
      /// \param[in] _msg Message containing link data
      private: void OnLinkData(ConstLinkDataPtr &_msg);

      /// \brief Returns the angular velocity.
      /// \return Angular velocity.
      public: math::Vector3 GetAngularVelocity() const;

      /// \brief Returns the imu linear acceleration
      /// \return Linear acceleration.
      public: math::Vector3 GetLinearAcceleration() const;

      /// \brief get orientation of the IMU relative to the reference pose
      /// \return returns the orientation quaternion of the IMU relative to
      /// the imu reference pose.
      public: math::Quaternion GetOrientation() const;

      /// \brief Sets the current pose as the IMU reference pose
      public: void SetReferencePose();

      // Documentation inherited.
      public: virtual bool IsActive();

      /// \brief Imu reference pose
      private: math::Pose referencePose;

      /// \brief Save previous imu linear velocity for computing acceleration.
      private: math::Vector3 lastLinearVel;

      /// \brief Imu linear acceleration
      private: math::Vector3 linearAcc;

      /// \brief store gravity vector to be added to the imu output.
      private: math::Vector3 gravity;

      /// \brief Imu data publisher
      private: transport::PublisherPtr pub;

      /// \brief Subscriber to link data published by parent entity
      private: transport::SubscriberPtr linkDataSub;

      /// \brief Parent entity which the Imu is attached to
      private: physics::LinkPtr parentEntity;

      /// \brief Imu message
      private: msgs::IMU imuMsg;

      /// \brief Mutex to protect reads and writes.
      private: mutable boost::mutex mutex;

      typedef std::list<boost::shared_ptr<msgs::LinkData const> >
          LinkDataMsgs_L;

      /// \brief Buffer for storing link data
      private: LinkDataMsgs_L incomingLinkData;
    };
    /// \}
  }
}
#endif
