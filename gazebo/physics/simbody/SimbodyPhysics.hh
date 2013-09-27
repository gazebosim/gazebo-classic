/*
 * Copyright 2013 Open Source Robotics Foundation
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

#ifndef _SIMBODY_PHYSICS_HH
#define _SIMBODY_PHYSICS_HH
#include <string>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/Shape.hh"
#include "gazebo/physics/simbody/SimbodyTypes.hh"

#include "gazebo/physics/simbody/simbody_inc.h"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_simbody Simbody Physics
    /// \{

    /// \brief Simbody physics engine
    class SimbodyPhysics : public PhysicsEngine
    {
      /// \brief Constructor
      public: SimbodyPhysics(WorldPtr _world);

      /// \brief Destructor
      public: virtual ~SimbodyPhysics();

      // Documentation inherited
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited
      public: virtual void Init();

      // Documentation inherited
      public: virtual void Reset();

      /// \brief Add a Model to the Simbody system.
      /// \param[in] _model Pointer to the model to add into Simbody.
      public: void InitModel(const physics::ModelPtr _model);

      // Documentation inherited
      public: virtual void InitForThread();

      // Documentation inherited
      public: virtual void UpdateCollision();

      // Documentation inherited
      public: virtual void UpdatePhysics();

      // Documentation inherited
      public: virtual void Fini();

      // Documentation inherited
      public: virtual std::string GetType() const;

      // Documentation inherited
      public: virtual LinkPtr CreateLink(ModelPtr _parent);

      // Documentation inherited
      public: virtual CollisionPtr CreateCollision(const std::string &_type,
                                                   LinkPtr _body);

      // Documentation inherited
      public: virtual JointPtr CreateJoint(const std::string &_type,
                                           ModelPtr _parent);

      // Documentation inherited
      public: virtual ShapePtr CreateShape(const std::string &_shapeType,
                                           CollisionPtr _collision);

      /// \brief Register a joint with the dynamics world
      public: SimTK::MultibodySystem *GetDynamicsWorld() const;

      // Documentation inherited
      public: virtual void SetGravity(const gazebo::math::Vector3 &_gravity);

      // Documentation inherited
      public: virtual void DebugPrint() const;

      // Documentation inherited
      public: virtual void SetSeed(uint32_t _seed);

      // Documentation inherited
      public: virtual ModelPtr CreateModel(BasePtr _parent);

      public: static SimTK::Quaternion QuadToQuad(const math::Quaternion &_q);

      public: static math::Quaternion QuadToQuad(const SimTK::Quaternion &_q);

      public: static SimTK::Vec3 Vector3ToVec3(const math::Vector3 &_v);

      public: static math::Vector3 Vec3ToVector3(const SimTK::Vec3 &_v);

      /// \brief Convert the given pose in x,y,z,thetax,thetay,thetaz format to
      /// a Simbody Transform. The rotation angles are interpreted as a
      /// body-fixed sequence, meaning we rotation about x, then about
      /// the new y, then about the now twice-rotated z.
      public: static SimTK::Transform Pose2Transform(const math::Pose &_pose);

      /// \brief Convert a Simbody transform to a pose in x,y,z,
      /// thetax,thetay,thetaz format.
      public: static math::Pose Transform2Pose(const SimTK::Transform &_xAB);

      /// \brief If the given element contains a <pose> element, return it as a
      /// Transform. Otherwise return the identity Transform. If there
      /// is more than one <pose> element, only the first one is processed.
      public: static SimTK::Transform GetPose(sdf::ElementPtr _element);

      public: static std::string GetTypeString(unsigned int _type);

      public: static std::string GetTypeString(physics::Base::EntityType _type);

      protected: virtual void OnRequest(ConstRequestPtr &_msg);

      protected: virtual void OnPhysicsMsg(ConstPhysicsPtr &_msg);

      /// \brief Helper functions
      private: void CreateMultibodyGraph(
        SimTK::MultibodyGraphMaker& _mbgraph, const physics::ModelPtr _model);

      /// \brief Initialize an empty simbody system
      private: void InitSimbodySystem();

      /// \brief Add Model to simbody system, and reinitialize state
      private: void AddStaticModelToSimbodySystem(
                   const physics::ModelPtr _model);

      private: void AddDynamicModelToSimbodySystem(
        const SimTK::MultibodyGraphMaker &_mbgraph,
        const physics::ModelPtr _model);

      /// \brief helper function for building SimbodySystem
      private: void AddCollisionsToLink(const physics::SimbodyLink *_link,
        SimTK::MobilizedBody &_mobod, SimTK::ContactCliqueId _modelClique);

      public: SimTK::MultibodySystem system;
      public: SimTK::SimbodyMatterSubsystem matter;
      public: SimTK::GeneralForceSubsystem forces;
      public: SimTK::Force::Gravity gravity;
      public: SimTK::Force::DiscreteForces discreteForces;
      public: SimTK::ContactTrackerSubsystem tracker;
      public: SimTK::CompliantContactSubsystem contact;
      public: SimTK:: Integrator *integ;

      /// \brief true if initialized
      public: bool simbodyPhysicsInitialized;

      public: bool simbodyPhysicsStepped;

      /// \brief contact material stiffness.  See sdf description for details.
      private: double contactMaterialStiffness;

      /// \brief contact material dissipation.  See sdf description for details.
      private: double contactMaterialDissipation;

      /// \brief contact material plastic coefficient of restitution.
      /// See sdf description for details.
      private: double contactMaterialPlasticCoefRestitution;

      /// \brief contact material plastic impact velocity.
      /// See sdf description for details.
      private: double contactMaterialPlasticImpactVelocity;

      /// \brief contact material static friction.
      /// See sdf description for details.
      private: double contactMaterialStaticFriction;

      /// \brief contact material dynamic friction.
      /// See sdf description for details.
      private: double contactMaterialDynamicFriction;

      /// \brief contact material viscous friction.
      /// See sdf description for details.
      private: double contactMaterialViscousFriction;

      /// \brief contact impact capture velocity.
      /// See sdf description for details.
      private: double contactImpactCaptureVelocity;

      /// \brief contact stiction transition velocity
      /// See sdf description for details.
      private: double contactStictionTransitionVelocity;

      private: SimTK::MultibodySystem *dynamicsWorld;

      private: common::Time lastUpdateTime;

      private: double stepTimeDouble;
    };
  /// \}
  }
}
#endif
