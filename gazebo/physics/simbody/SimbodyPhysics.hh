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
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_simbody Simbody Physics
    /// \{

    /// \brief Simbody physics engine
    class GAZEBO_VISIBLE SimbodyPhysics : public PhysicsEngine
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

      /// \brief Convert gazebo::math::Quaternion to SimTK::Quaternion
      /// \param[in] _q Gazeb's math::Quaternion object
      /// \return Simbody's SimTK::Quaternion object
      public: static SimTK::Quaternion QuadToQuad(const math::Quaternion &_q);

      /// \brief Convert SimTK::Quaternion to gazebo::math::Quaternion
      /// \param[in] _q Simbody's SimTK::Quaternion object
      /// \return Gazeb's math::Quaternion object
      public: static math::Quaternion QuadToQuad(const SimTK::Quaternion &_q);

      /// \brief Convert gazebo::math::Vector3 to SimTK::Vec3
      /// \param[in] _v Gazeb's math::Vector3 object
      /// \return Simbody's SimTK::Vec3 object
      public: static SimTK::Vec3 Vector3ToVec3(const math::Vector3 &_v);

      /// \brief Convert SimTK::Vec3 to gazebo::math::Vector3
      /// \param[in] _v Simbody's SimTK::Vec3 object
      /// \return Gazeb's math::Vector3 object
      public: static math::Vector3 Vec3ToVector3(const SimTK::Vec3 &_v);

      /// \brief Convert the given pose in x,y,z,thetax,thetay,thetaz format to
      /// a Simbody Transform. The rotation angles are interpreted as a
      /// body-fixed sequence, meaning we rotation about x, then about
      /// the new y, then about the now twice-rotated z.
      /// \param[in] _pose Gazeb's math::Pose object
      /// \return Simbody's SimTK::Transform object
      public: static SimTK::Transform Pose2Transform(const math::Pose &_pose);

      /// \brief Convert a Simbody transform to a pose in x,y,z,
      /// thetax,thetay,thetaz format.
      /// \param[in] _xAB Simbody's SimTK::Transform object
      /// \return Gazeb's math::Pose object
      public: static math::Pose Transform2Pose(const SimTK::Transform &_xAB);

      /// \brief If the given element contains a <pose> element, return it as a
      /// Transform. Otherwise return the identity Transform. If there
      /// is more than one <pose> element, only the first one is processed.
      public: static SimTK::Transform GetPose(sdf::ElementPtr _element);

      /// \brief Convert Base::GetType() to string,
      /// this is needed by the MultibodyGraphMaker.
      /// \param[in] _type Joint type returned by Joint::GetType().
      /// \return a hard-coded string needed by the MultibodyGraphMaker.
      public: static std::string GetTypeString(unsigned int _type);

      /// \brief Convert Base::GetType() to string,
      /// this is needed by the MultibodyGraphMaker.
      /// \param[in] _type Joint type returned by Joint::GetType().
      /// \return a hard-coded string needed by the MultibodyGraphMaker.
      public: static std::string GetTypeString(physics::Base::EntityType _type);

      // Documentation inherited
      protected: virtual void OnRequest(ConstRequestPtr &_msg);

      // Documentation inherited
      protected: virtual void OnPhysicsMsg(ConstPhysicsPtr &_msg);

      /// \brief CREATE MULTIBODY GRAPH
      /// Define Gazebo joint types, then use links and joints in the
      /// given model to construct a reasonable spanning-tree-plus-constraints
      /// multibody graph to represent that model. An exception will be
      /// thrown if this fails.  Note that this step is not Simbody dependent.
      /// \param[in] _mbgraph Create a MultibodyGraphMaker that is equivalent
      /// of incoming physics::Model.
      /// \param[in] _model Model loaded by Gazebo parsing SDF.
      private: void CreateMultibodyGraph(
        SimTK::MultibodyGraphMaker& _mbgraph, const physics::ModelPtr _model);

      /// \brief BUILD SIMBODY SYSTEM
      /// Given a desired multibody graph, gravity, and the Gazebo model
      /// that was used to generate the graph, create a Simbody System
      /// for it. There are many limitations here, especially in the
      /// handling of contact. Any Gazebo features that we haven't
      /// modeled are just ignored.  The GazeboModel is updated so that
      /// its links and joints have references to their corresponding
      /// Simbody elements.  We set up some visualization here so we
      /// can see what's happening but this would not be needed in Gazebo
      /// since it does its own visualization.
      private: void InitSimbodySystem();

      /// \brief Add a static Model to simbody system, and reinitialize state
      /// \param[in] _model the incoming static Gazebo physics::Model.
      private: void AddStaticModelToSimbodySystem(
                   const physics::ModelPtr _model);

      /// \brief Read from MultibodydGraphMaker and construct a physics::Model.
      /// \param[in] _mbgraph Contain MultibodyGraphMaker object.
      /// \param[in] _model Pointer to gazebo model, not used at this time.
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

      // Documentation inherited
      public: virtual boost::any GetParam(const std::string &_key) const;

      // Documentation inherited
      public: virtual bool SetParam(const std::string &_key,
                  const boost::any &_value);

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

      /// \brief The type of the solver.
      /// Not used, just getting ready for optional pgs rigid contacts.
      private: std::string solverType;

      /// \brief The type of integrator:
      ///   SimTK::RungeKuttaMersonIntegrator(system)
      ///   SimTK::RungeKutta3Integrator(system)
      ///   SimTK::RungeKutta2Integrator(system)
      ///   SimTK::SemiExplicitEuler2Integrator(system)
      private: std::string integratorType;
    };
  /// \}
  }
}
#endif
