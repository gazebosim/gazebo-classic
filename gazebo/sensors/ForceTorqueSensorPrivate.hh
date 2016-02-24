/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_SENSORS_FORCETORQUESENSOR_PRIVATE_HH_
#define _GAZEBO_SENSORS_FORCETORQUESENSOR_PRIVATE_HH_

#include <mutex>
#include <ignition/math/Matrix3.hh>

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \internal
    /// \brief Force torque sensor private data.
    class ForceTorqueSensorPrivate
    {
      /// \brief Update event.
      public: event::EventT<void(msgs::WrenchStamped)> update;

      /// \brief Parent joint, from which we get force torque info.
      public: physics::JointPtr parentJoint;

      /// \brief Publishes the wrenchMsg.
      public: transport::PublisherPtr wrenchPub;

      /// \brief Message the store the current force torque info.
      public: msgs::WrenchStamped wrenchMsg;

      /// \brief Mutex to protect the wrench message
      public: std::mutex mutex;

      /// \brief Which orientation we support for returning sensor measure
      public: enum MeasureFrame
      {
        PARENT_LINK,
        CHILD_LINK,
        SENSOR
      };

      /// \brief Frame in which we return the measured force torque info.
      public: MeasureFrame measureFrame;

      /// \brief Direction of the measure
      ///        True if the measured force torque is the one applied
      ///        by the parent on the child, false otherwise
      public: bool parentToChild;

      /// \brief Rotation matrix than transforms a vector expressed in child
      ///        orientation in a vector expressed in joint orientation.
      ///        Necessary is the measure is specified in joint frame.
      public: ignition::math::Matrix3d rotationSensorChild;
    };
  }
}
#endif
