/*
 * Copyright 2012 Nate Koenig
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

      // Documentation inherited
      protected: virtual void Load(sdf::ElementPtr _node);

      // Documentation inherited
      protected: virtual void Init();

      // Documentation inherited
      protected: virtual void Update();

      // Documentation inherited
      protected: virtual void Fini();

      /// \brief Returns velocity as a math::Pose
      /// \return velocity data stored in Pose
      public: math::Pose GetVelocity();

      private: Pose prevPose;
      private: Pose imuVel;
    };
    /// \}
  }
}
#endif
