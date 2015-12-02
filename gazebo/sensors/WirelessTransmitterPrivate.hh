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
#ifndef _GAZEBO_WIRELESS_TRANSMITTER_PRIVATE_HH_
#define _GAZEBO_WIRELESS_TRANSMITTER_PRIVATE_HH_

#include <string>
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/sensors/WirelessTransceiverPrivate.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \internal
    /// \brief Wireless transmitter private data
    class WirelessTransmitterPrivate : public WirelessTransceiverPrivate
    {
      /// \brief Constructor
      public: WirelessTransmitterPrivate() :
              visualize(false),
              essid("MyESSID"),
              freq(2442.0)
      {
      }

      /// \brief Constant used in the propagation model when there are no
      /// obstacles between transmitter and receiver
      public: static const double NEmpty;

      /// \brief Constant used in the propagation model when there are
      /// obstacles between transmitter and receiver
      public: static const double NObstacle;

      /// \brief Std dev of the Gaussian random variable used in the
      /// propagation model
      public: static const double ModelStdDev;

      /// \brief Size of the grid used for visualization.
      public: static const double Step;

      /// \brief The visualization shows the propagation model using a circular
      /// grid, where the maximum radius covered is MaxRadius
      public: static const double MaxRadius;

      // \brief When true it will publish the propagation grid to be used
      // by the transmitter visual layer
      public: bool visualize;

      /// \brief Service Set Identifier (network name).
      public: std::string essid;

      /// \brief Reception frequency (MHz).
      public: double freq;

      // \brief Ray used to test for collisions when placing entities
      public: physics::RayShapePtr testRay;
    };
  }
}
#endif
