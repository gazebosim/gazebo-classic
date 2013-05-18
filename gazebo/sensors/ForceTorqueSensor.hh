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

#ifndef _FORCE_TORQAUE_SENSOR_HH_
#define _FORCE_TORQAUE_SENSOR_HH_

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

    /// \class ForceTorqueSensor ForceTorqueSensor.hh sensors/sensors.hh
    /// \brief An FORCE_TORQAUE_ sensor.
    class ForceTorqueSensor: public Sensor
    {
      /// \brief Constructor.
      public: ForceTorqueSensor();

      /// \brief Destructor.
      public: virtual ~ForceTorqueSensor();

      // Documentation inherited.
      protected: void Load(const std::string &_worldName, sdf::ElementPtr _sdf);

      // Documentation inherited.
      protected: virtual void Load(const std::string &_worldName);

      /// \brief Initialize the ForceTorque sensor.
      public: virtual void Init();

      // Documentation inherited
      protected: virtual void UpdateImpl(bool _force);

      // Documentation inherited
      protected: virtual void Fini();

      /// \brief Returns the forceTorque message
      /// \return ForceTorque message.
      public: msgs::ForceTorque GetForceTorqueMessage() const;

      // Documentation inherited.
      public: virtual bool IsActive();

      /// \brief Callback when link data is received
      /// \param[in] _msg Message containing link data
      private: void OnLinkData(ConstLinkDataPtr &_msg);

      /// \brief ForceTorque reference pose
      private: math::Pose referencePose;

      /// \brief ForceTorque data publisher
      private: transport::PublisherPtr pub;

      /// \brief Subscriber to link data published by parent entity
      private: transport::SubscriberPtr linkDataSub;

      /// \brief Parent entity which the ForceTorque is attached to
      private: physics::LinkPtr parentEntity;

      /// \brief ForceTorque message
      private: msgs::ForceTorque forceTorqueMsg;

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

      /// \brief If true, apply the noise model specified by other noise
      /// parameters
      private: bool noiseActive;

      /// \brief Which type of noise we're applying
      private: enum NoiseModelType noiseType;

      /// \brief If noiseType==GAUSSIAN, the mean of the distibution
      /// from which we sample when adding noise to data
      private: double noiseMean;

      /// \brief If noiseType==GAUSSIAN, the standard devation of the
      /// distibution from which we sample when adding noise to data
      private: double noiseStdDev;

      /// \brief If noiseType==GAUSSIAN, the bias we'll add to data
      private: double bias;
    };
    /// \}
  }
}
#endif
