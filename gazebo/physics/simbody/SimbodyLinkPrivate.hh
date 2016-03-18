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
#ifndef _GAZEBO_PHYSICS_SIMBODY_SIMBODYJOINTPRIVATE_HH_
#define _GAZEBO_PHYSICS_SIMBODY_SIMBODYJOINTPRIVATE_HH_

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Private data for SimbodyLink
    class SimbodyLinkPrivate
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
      private: bool gravityMode;

      /// \brief Trigger setting of link according to staticLink.
      private: bool staticLinkDirty;

      /// \brief Trigger setting of link gravity mode
      private: bool gravityModeDirty;

      /// \brief If true, freeze link to world (inertial) frame.
      private: bool staticLink;

      /// \brief Event connection for SetLinkStatic
      private: event::ConnectionPtr staticLinkConnection;

      /// \brief Event connection for SetGravityMode
      private: event::ConnectionPtr gravityModeConnection;

      /// \brief save simbody free state for reconstructing simbody model graph
      private: std::vector<double> simbodyQ;

      /// \brief save simbody free state for reconstructing simbody model graph
      private: std::vector<double> simbodyU;

      /// \brief keep a pointer to the simbody physics engine for convenience
      private: SimbodyPhysicsPtr simbodyPhysics;
    };
  }
}
