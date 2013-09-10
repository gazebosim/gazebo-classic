/*
 * Copyright 2012 Open Source Robotics Foundation
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
/* Desc: The Simbody physics engine wrapper
 * Author: Nate Koenig
 * Date: 11 June 2009
 */

#ifndef SIMBODYPHYSICS_HH
#define SIMBODYPHYSICS_HH
#include <string>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <Simbody.h>

#include "gazebo/physics/simbody/simbody_inc.h"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/Shape.hh"
#include "gazebo/physics/simbody/SimbodyTypes.hh"

namespace gazebo
{
  namespace physics
  {
    class Entity;
    class XMLConfigNode;
    class Mass;

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

      /// \brief Load the Simbody engine
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Initialize the Simbody engine
      public: virtual void Init();

      // Documentation inherited
      public: virtual void Reset();

      /// \brief Add a Model to the Simbody system
      public: void InitModel(const physics::Model* _model);

      /// \brief Init the engine for threads.
      public: virtual void InitForThread();

      /// \brief Update the Simbody collision
      public: virtual void UpdateCollision();

      /// \brief Update the Simbody engine
      public: virtual void UpdatePhysics();

      /// \brief Finilize the Simbody engine
      public: virtual void Fini();

      // Documentation inherited
      public: virtual std::string GetType() const
                      { return "simbody"; }

      /// \brief Set the simulation step time
      public: virtual void SetStepTime(double _value);

      /// \brief Get the simulation step time
      public: virtual double GetStepTime();

      /// \brief Create a new body
      public: virtual LinkPtr CreateLink(ModelPtr _parent);

      /// \brief Create a new collision
      public: virtual CollisionPtr CreateCollision(const std::string &_type,
                                                   LinkPtr _body);

      /// \brief Create a new joint
      public: virtual JointPtr CreateJoint(const std::string &_type,
                                           ModelPtr _parent);

      public: virtual ShapePtr CreateShape(const std::string &_shapeType,
                                           CollisionPtr _collision);

      /// \brief Create a physics based ray sensor
      // public: virtual PhysicsRaySensor *CreateRaySensor(Link *body);

      /// \brief Convert an simbody mass to a gazebo Mass
      public: virtual void ConvertMass(InertialPtr _inertial,
                                       void *_engineMass);

      /// \brief Convert an gazebo Mass to a simbody Mass
      public: virtual void ConvertMass(void *_engineMass,
                                       InertialPtr _inertial);

      /// \brief Register a joint with the dynamics world
      public: SimTK::MultibodySystem *GetDynamicsWorld() const
              {return this->dynamicsWorld;}

      /// \brief Set the gavity vector
      public: virtual void SetGravity(const gazebo::math::Vector3 &gravity);

      public: virtual void DebugPrint() const;

      // Documentation inherited
      public: virtual void SetSeed(uint32_t _seed);

      // Documentation inherited
      public: virtual ModelPtr CreateModel(BasePtr _parent);
      
      private: SimTK::MultibodySystem *dynamicsWorld;

      private: common::Time lastUpdateTime;

      private: double stepTimeDouble;




      public: SimTK::MultibodySystem system;
      public: SimTK::SimbodyMatterSubsystem matter;
      public: SimTK::GeneralForceSubsystem forces;
      public: SimTK::Force::Gravity gravity;
      public: SimTK::Force::DiscreteForces discreteForces;
      public: SimTK::ContactTrackerSubsystem tracker;
      public: SimTK::CompliantContactSubsystem contact;
      public: SimTK:: Integrator *integ;

      public: static SimTK::Quaternion QuadToQuad(const math::Quaternion & _q)
      {
        return SimTK::Quaternion(_q.w, _q.x, _q.y, _q.z);
      }

      public: static math::Quaternion QuadToQuad(const SimTK::Quaternion & _q)
      {
        return math::Quaternion(_q[0], _q[1], _q[2], _q[3]);
      }

      public: static SimTK::Vec3 Vector3ToVec3(const math::Vector3& _v)
      {
        return SimTK::Vec3(_v.x, _v.y, _v.z);
      }

      public: static math::Vector3 Vec3ToVector3(const SimTK::Vec3& _v)
      {
        return math::Vector3(_v[0], _v[1], _v[2]);
      }

      // Convert the given pose in x,y,z,thetax,thetay,thetaz format to
      // a Simbody Transform. The rotation angles are interpreted as a
      // body-fixed sequence, meaning we rotation about x, then about
      // the new y, then about the now twice-rotated z.
      public: static SimTK::Transform Pose2Transform(const math::Pose& _pose)
      {
        SimTK::Quaternion q(_pose.rot.w, _pose.rot.x, _pose.rot.y,
                         _pose.rot.z);
        SimTK::Vec3 v(_pose.pos.x, _pose.pos.y, _pose.pos.z);
        SimTK::Transform frame(SimTK::Rotation(q), v); 
        return frame;
      }

      // Convert a Simbody transform to a pose in x,y,z,thetax,thetay,thetaz
      // format.
      public: static math::Pose Transform2Pose(const SimTK::Transform& X_AB)
      {
        SimTK::Quaternion q(X_AB.R());
        const SimTK::Vec4 &qv = q.asVec4();
        return math::Pose(math::Vector3(X_AB.p()[0], X_AB.p()[1], X_AB.p()[2]),
          math::Quaternion(qv[0], qv[1], qv[2], qv[3]));
      }

      // If the given element contains a <pose> element, return it as a
      // Transform. Otherwise return the identity Transform. If there
      // is more than one <pose> element, only the first one is processed.
      public: static SimTK::Transform GetPose(sdf::ElementPtr _element)
      {
        const math::Pose pose = _element->Get<math::Pose>("pose");
        return Pose2Transform(pose);
      }

      public: static std::string GetTypeString(unsigned int _type)
      {
        return SimbodyPhysics::GetTypeString(physics::Base::EntityType(_type));
      }

      public: static std::string GetTypeString(physics::Base::EntityType _type);

      protected: virtual void OnRequest(ConstRequestPtr &_msg);

      protected: virtual void OnPhysicsMsg(ConstPhysicsPtr &_msg);

      /// \brief true if initialized
      public: bool simbodyPhysicsInitialized;
      public: bool simbodyPhysicsStepped;

      /// \brief Helper functions
      private: void CreateMultibodyGraph(
        SimTK::MultibodyGraphMaker& _mbgraph, const physics::Model* _model);

      /// \brief Initialize an empty simbody system
      private: void InitSimbodySystem();

      /// \brief Add Model to simbody system, and reinitialize state
      private: void AddStaticModelToSimbodySystem(const physics::Model* _model);
      private: void AddDynamicModelToSimbodySystem(
        const SimTK::MultibodyGraphMaker& _mbgraph,
        const physics::Model* _model);

      /// \brief helper function for building SimbodySystem
      private: void AddCollisionsToLink(const physics::SimbodyLink* _link,
        SimTK::MobilizedBody &_mobod, SimTK::ContactCliqueId _modelClique);

      
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
    };

  /// \}
  }
}
#endif
