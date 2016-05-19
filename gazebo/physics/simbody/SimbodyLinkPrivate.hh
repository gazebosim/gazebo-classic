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
#ifndef GAZEBO_PHYSICS_SIMBODY_SIMBODYLINKPRIVATE_HH_
#define GAZEBO_PHYSICS_SIMBODY_SIMBODYLINKPRIVATE_HH_

#include <vector>

#include "gazebo/physics/LinkPrivate.hh"
#include "gazebo/physics/simbody/simbody_inc.h"

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Private data for SimbodyLink
    class SimbodyLinkPrivate : public LinkPrivate
    {
      /// \brief: Force this link to be a base body, where its inboard
      /// body is the world with 6DOF.
      public: bool mustBeBaseLink;

      // Below to be filled in after everything is loaded
      // Which MobilizedBody corresponds to the master instance of this link.
      public: SimTK::MobilizedBody masterMobod;

      // Keeps track if physics has been initialized
      public: bool physicsInitialized;

      // If this link got split into a master and slaves, these are the
      // MobilizedBodies used to mobilize the slaves.
      public: std::vector<SimTK::MobilizedBody> slaveMobods;

      // And these are the Weld constraints used to attach slaves to master.
      public: std::vector<SimTK::Constraint::Weld> slaveWelds;

      /// \brief store gravity mode given link might not be around
      public: bool gravityMode;

      /// \brief Trigger setting of link according to staticLink.
      public: bool staticLinkDirty;

      /// \brief Trigger setting of link gravity mode
      public: bool gravityModeDirty;

      /// \brief If true, freeze link to world (inertial) frame.
      public: bool staticLink;

      /// \brief Event connection for SetLinkStatic
      public: event::ConnectionPtr staticLinkConnection;

      /// \brief Event connection for SetGravityMode
      public: event::ConnectionPtr gravityModeConnection;

      /// \brief save simbody free state for reconstructing simbody model graph
      public: std::vector<double> simbodyQ;

      /// \brief save simbody free state for reconstructing simbody model graph
      public: std::vector<double> simbodyU;

      /// \brief keep a pointer to the simbody physics engine for convenience
      public: SimbodyPhysicsPtr simbodyPhysics;
    };
  }
}
#endif
