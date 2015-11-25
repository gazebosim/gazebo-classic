/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef _GAZEBO_RAY_SENSOR_PRIVATE_HH_
#define _GAZEBO_RAY_SENSOR_PRIVATE_HH_

#include <mutex>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/sensors/SensorPrivate.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \internal
    /// \brief Ray sensor private data.
    class RaySensorPrivate : public SensorProtected
    {
      /// \brief Laser collision pointer.
      public: physics::CollisionPtr laserCollision;

      /// \brief Multi ray shapre pointer.
      public: physics::MultiRayShapePtr laserShape;

      /// \brief Parent entity pointer
      public: physics::EntityPtr parentEntity;

      /// \brief Publisher for the scans
      public: transport::PublisherPtr scanPub;

      /// \brief Mutex to protect laserMsg
      public: std::mutex mutex;

      /// \brief Laser message.
      public: msgs::LaserScanStamped laserMsg;
    };
  }
}
#endif
