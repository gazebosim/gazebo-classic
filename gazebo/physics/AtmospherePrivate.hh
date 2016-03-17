/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#ifndef _GAZEBO_PHYSICS_ATMOSPHEREPRIVATE_HH_
#define _GAZEBO_PHYSICS_ATMOSPHEREPRIVATE_HH_

#include <sdf/sdf.hh>

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  namespace physics
  {
    class AtmospherePrivate
    {
      /// \brief Our SDF values.
      public: sdf::ElementPtr sdf;

      /// \brief Pointer to the world.
      public: WorldPtr world;

      /// \brief Node for communication.
      public: transport::NodePtr node;

      /// \brief Response publisher.
      public: transport::PublisherPtr responsePub;

      /// \brief Subscribe to the atmosphere topic.
      public: transport::SubscriberPtr atmosphereSub;

      /// \brief Subscribe to the request topic.
      public: transport::SubscriberPtr requestSub;

      /// \brief Temperature at sea level in kelvins.
      public: double temperature = 288.15;

      /// \brief Temperature gradient at sea level in K/m.
      public: double temperatureGradient = -0.0065;

      /// \brief Pressure of the air at sea level in pascals.
      public: double pressure = 101325;

      /// \brief Mass density of the air at sea level in kg/m^3.
      public: double massDensity = 1.225;
    };
  }
}

#endif
