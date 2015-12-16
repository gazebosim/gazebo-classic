/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#ifndef _GAZEBO_PHYSICS_ODE_ODEJOINT_PRIVATE_HH_
#define _GAZEBO_PHYSICS_ODE_ODEJOINT_PRIVATE_HH_

#include "gazebo/common/Time.hh"

#include "gazebo/physics/ode/ode_inc.h"
#include "gazebo/physics/JointPrivate.hh"

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief ODE joint protected class
    class ODEJointProtected : public JointProtected
    {
      /// \brief This is our ODE ID
      public: dJointID jointId;
    };

    /// \internal
    /// \brief ODE joint private class
    class ODEJointPrivate
    {
      /// \brief Feedback data for this joint
      public: dJointFeedback *feedback;

      /// \brief CFM for joint's limit constraint
      public: double stopCFM;

      /// \brief ERP for joint's limit constraint
      public: double stopERP;

      /// \brief Save force applied by user
      /// This plus the joint feedback (joint contstraint forces) is the
      /// equivalent of simulated force torque sensor reading
      /// Allocate a 2 vector in case hinge2 joint is used.
      /// This is used by ODE to store external force applied by the user.
      public: std::array<double, MAX_JOINT_AXIS> forceApplied;

      /// \brief Save time at which force is applied by user
      /// This will let us know if it's time to clean up forceApplied.
      public: common::Time forceAppliedTime;

      /// \brief internal variable to keep track of implicit damping internals
      public: std::array<int, MAX_JOINT_AXIS> implicitDampingState;

      /// \brief save current implicit damping coefficient
      public: std::array<double, MAX_JOINT_AXIS> currentKd;

      /// \brief save current implicit stiffness coefficient
      public: std::array<double, MAX_JOINT_AXIS> currentKp;

      /// \brief internal variable to keep track if ConnectJointUpdate
      /// has been called on a damping method
      public: bool stiffnessDampingInitialized;

      /// \brief flag to use implicit joint stiffness damping if true.
      public: bool useImplicitSpringDamper;
    };
  }
}
#endif
