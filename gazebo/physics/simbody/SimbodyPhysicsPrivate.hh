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
#ifndef GAZEBO_PHYSICS_SIMBODY_SIMBODYPHYSICSPRIVATE_HH
#define GAZEBO_PHYSICS_SIMBODY_SIMBODYPHYSICSPRIVATE_HH

#include "gazebo/physics/PhysicsEnginePrivate.hh"

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Private physics data
    class SimbodyPhysicsPrivate : public PhysicsEnginePrivate
    {
      /// \brief Constructor
      public: SimbodyPhysicsPrivate()
              : system(), matter(system), forces(system),
              gravity(forces, matter, -SimTK::ZAxis, 0),
              discreteForces(forces, matter),
              tracker(system), contact(system, tracker)
      {
      }

      /// \brief true if initialized
      public: bool simbodyPhysicsInitialized;

      public: bool simbodyPhysicsStepped;

      public: SimTK::MultibodySystem system;
      public: SimTK::SimbodyMatterSubsystem matter;
      public: SimTK::GeneralForceSubsystem forces;
      public: SimTK::Force::Gravity gravity;
      public: SimTK::Force::DiscreteForces discreteForces;
      public: SimTK::ContactTrackerSubsystem tracker;
      public: SimTK::CompliantContactSubsystem contact;
      public: SimTK::Integrator *integ = NULL;

      /// \brief contact material stiffness.  See sdf description for details.
      public: double contactMaterialStiffness = 0.0;

      /// \brief contact material dissipation.  See sdf description for details.
      public: double contactMaterialDissipation = 0.0;

      /// \brief contact material plastic coefficient of restitution.
      /// See sdf description for details.
      public: double contactMaterialPlasticCoefRestitution = 0.0;

      /// \brief contact material plastic impact velocity.
      /// See sdf description for details.
      public: double contactMaterialPlasticImpactVelocity = 0.0;

      /// \brief contact material static friction.
      /// See sdf description for details.
      public: double contactMaterialStaticFriction = 0.0;

      /// \brief contact material dynamic friction.
      /// See sdf description for details.
      public: double contactMaterialDynamicFriction = 0.0;

      /// \brief contact material viscous friction.
      /// See sdf description for details.
      public: double contactMaterialViscousFriction = 0.0;

      /// \brief contact impact capture velocity.
      /// See sdf description for details.
      public: double contactImpactCaptureVelocity = 0.0;

      /// \brief contact stiction transition velocity
      /// See sdf description for details.
      public: double contactStictionTransitionVelocity = 0.0;

      public: SimTK::MultibodySystem *dynamicsWorld = NULL;

      public: common::Time lastUpdateTime;

      public: double stepTimeDouble = 0.0;

      /// \brief The type of the solver.
      /// Not used, just getting ready for optional pgs rigid contacts.
      public: std::string solverType;

      /// \brief The type of integrator:
      ///   SimTK::RungeKuttaMersonIntegrator(system)
      ///   SimTK::RungeKutta3Integrator(system)
      ///   SimTK::RungeKutta2Integrator(system)
      ///   SimTK::SemiExplicitEuler2Integrator(system)
      public: std::string integratorType;
    };
  }
}
#endif
