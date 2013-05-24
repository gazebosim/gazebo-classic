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
      /// \brief Constructor
      public: ForceTorqueSensor();

      /// \brief Destructor
      public: virtual ~ForceTorqueSensor();

      // Documentation inherited
      public: virtual void Load(const std::string &_worldName);

      // Documentation inherited
      public: virtual void Init();

      // Documentation inherited
      protected: virtual void UpdateImpl(bool _force);

      // Documentation inherited
      protected: virtual void Fini();

      // Documentation inherited
      public: virtual std::string GetTopic() const;

      // Documentation inherited
      public: virtual bool IsActive();

      private: physics::JointPtr parentJoint;

      private: transport::PublisherPtr wrenchPub;

      private: boost::mutex mutex;
    };
    /// \}
  }
}
#endif
