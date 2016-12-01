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
#ifndef GAZEBO_RENDERING_LASERVISUAL_PRIVATE_HH_
#define GAZEBO_RENDERING_LASERVISUAL_PRIVATE_HH_

#include <vector>

#include "gazebo/msgs/MessageTypes.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/rendering/VisualPrivate.hh"

namespace gazebo
{
  namespace rendering
  {
    class DynamicLines;

    /// \brief Private data for the Laser Visual class.
    class LaserVisualPrivate : public VisualPrivate
    {
      /// \brief Pointer to a node that handles communication.
      public: transport::NodePtr node;

      /// \brief Subscription to the laser data.
      public: transport::SubscriberPtr laserScanSub;

      /// \brief Renders the laser data as a triangle strip.
      public: std::vector<DynamicLines *> rayStrips;

      /// \brief Renders laser data for rays that do not hit obstacles.
      public: std::vector<DynamicLines *> noHitRayStrips;

      /// \brief Renders a deadzone that is between the sensor's origin
      /// and start of the rays.
      public: std::vector<DynamicLines *> deadzoneRayFans;

      /// \brief Renders the laser data as a line list.
      public: std::vector<DynamicLines *> rayLines;

      /// \brief Mutex to protect the contact message.
      public: boost::mutex mutex;

      /// \brief True if we have received a message.
      public: bool receivedMsg;

      /// \brief The current contact message.
      public: boost::shared_ptr<msgs::LaserScanStamped const> laserMsg;

      /// \brief Pre render connection.
      public: event::ConnectionPtr connection;
    };
  }
}
#endif
