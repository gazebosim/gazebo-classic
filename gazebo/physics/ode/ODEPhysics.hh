/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#ifndef _ODEPHYSICS_HH_
#define _ODEPHYSICS_HH_

#include <tbb/spin_mutex.h>
#include <tbb/concurrent_vector.h>
#include <string>
#include <utility>

#include <boost/thread/thread.hpp>

#include "gazebo/physics/ode/ode_inc.h"
#include "gazebo/physics/ode/ODETypes.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/Contact.hh"
#include "gazebo/physics/Shape.hh"
#include "gazebo/gazebo_config.h"
#include "gazebo/util/system.hh"

#include "gazebo/physics/ode/ODEPhysicsPrivate.hh"

namespace gazebo
{
  namespace physics
  {
    /// \brief ODE physics engine.
    class GAZEBO_VISIBLE ODEPhysics : public PhysicsEngine
    {
      /// \enum ODEParam
      /// \brief ODE Physics parameter types.
      public: enum ODEParam
      {
        /// \brief Solve type
        SOLVER_TYPE,

        /// \brief Constraint force mixing
        GLOBAL_CFM,

        /// \brief Error reduction parameter
        GLOBAL_ERP,

        /// \brief Number of iterations
        SOR_PRECON_ITERS,

        /// \brief Number of iterations
        PGS_ITERS,

        /// \brief SOR over-relaxation parameter
        SOR,

        /// \brief Max correcting velocity
        CONTACT_MAX_CORRECTING_VEL,

        /// \brief Surface layer depth
        CONTACT_SURFACE_LAYER,

        /// \brief Maximum number of contacts
        MAX_CONTACTS,

        /// \brief Minimum step size
        MIN_STEP_SIZE,

        /// \brief Limit ratios of inertias of adjacent links (note that the
        /// corresponding SDF tag is "use_dynamic_moi_rescaling")
        INERTIA_RATIO_REDUCTION
      };

      /// \brief Constructor.
      /// \param[in] _world The World that uses this physics engine.
      public: ODEPhysics(WorldPtr _world);

      /// \brief Destructor.
      public: virtual ~ODEPhysics();

      // Documentation inherited
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited
      public: virtual void Init();

      // Documentation inherited
      public: virtual void Reset();

      // Documentation inherited
      public: virtual void InitForThread();

      // Documentation inherited
      public: virtual void UpdateCollision();

      // Documentation inherited
      public: virtual void UpdatePhysics();

      // Documentation inherited
      public: virtual void Fini();

      // Documentation inherited
      public: virtual std::string GetType() const
                      { return "ode"; }

      // Documentation inherited
      public: virtual LinkPtr CreateLink(ModelPtr _parent);

      // Documentation inherited
      public: virtual CollisionPtr CreateCollision(
                  const std::string &_shapeType, LinkPtr _parent);

      // Documentation inherited
      public: virtual ShapePtr CreateShape(const std::string &_shapeType,
                                           CollisionPtr _collision);

      // Documentation inherited
      public: virtual JointPtr CreateJoint(const std::string &_type,
                                           ModelPtr _parent);

      // Documentation inherited
      public: virtual void SetGravity(const gazebo::math::Vector3 &_gravity);

      // Documentation inherited
      public: virtual void SetWorldCFM(double cfm);

      // Documentation inherited
      public: virtual void SetWorldERP(double erp);

      // Documentation inherited
      public: virtual void SetSORPGSPreconIters(unsigned int iters);

      // Documentation inherited
      public: virtual void SetSORPGSIters(unsigned int iters);

      // Documentation inherited
      public: virtual void SetSORPGSW(double w);

      // Documentation inherited
      public: virtual void SetContactMaxCorrectingVel(double vel);

      // Documentation inherited
      public: virtual void SetContactSurfaceLayer(double layer_depth);

      // Documentation inherited
      public: virtual void SetMaxContacts(unsigned int max_contacts);

      // Documentation inherited
      public: virtual double GetWorldCFM();

      // Documentation inherited
      public: virtual double GetWorldERP();

      // Documentation inherited
      public: virtual int GetSORPGSPreconIters();

      // Documentation inherited
      public: virtual int GetSORPGSIters();

      // Documentation inherited
      public: virtual double GetSORPGSW();

      // Documentation inherited
      public: virtual double GetContactMaxCorrectingVel();

      // Documentation inherited
      public: virtual double GetContactSurfaceLayer();

      // Documentation inherited
      public: virtual unsigned int GetMaxContacts();

      // Documentation inherited
      public: virtual void DebugPrint() const;

      // Documentation inherited
      public: virtual void SetSeed(uint32_t _seed);

      /// Documentation inherited
      public: virtual bool SetParam(const std::string &_key,
                  const boost::any &_value);

      /// Documentation inherited
      public: virtual boost::any GetParam(const std::string &_key) const;

      /// \brief Return the world space id.
      /// \return The space id for the world.
      public: dSpaceID GetSpaceId() const;

      /// \brief Get the world id.
      /// \return The world id.
      public: dWorldID GetWorldId();

      /// \brief Convert an ODE mass to Inertial.
      /// \param[out] _intertial Pointer to an Inertial object.
      /// \param[in] _odeMass Pointer to an ODE mass that will be converted.
      public: static void ConvertMass(InertialPtr _interial, void *_odeMass);

      /// \brief Convert an Inertial to ODE mass.
      /// \param[out] _odeMass Pointer to an ODE mass.
      /// \param[in] _intertial Pointer to an Inertial object that will be
      /// converted.
      public: static void ConvertMass(void *_odeMass, InertialPtr _inertial);

      /// \brief Get the step type (quick, world).
      /// \return The step type.
      public: virtual std::string GetStepType() const;

      /// \brief Set the step type (quick, world).
      /// \param[in] _type The step type (quick or world).
      public: virtual void SetStepType(const std::string &_type);


      /// \brief Collide two collision objects.
      /// \param[in] _collision1 First collision object.
      /// \param[in] _collision2 Second collision object.
      /// \param[in,out] _contactCollision Array of contacts.
      public: void Collide(ODECollision *_collision1, ODECollision *_collision2,
                           dContactGeom *_contactCollisions);

      /// \brief process joint feedbacks.
      /// \param[in] _feedback ODE Joint Contact feedback information.
      public: void ProcessJointFeedback(ODEJointFeedback *_feedback);

      protected: virtual void OnRequest(ConstRequestPtr &_msg);

      protected: virtual void OnPhysicsMsg(ConstPhysicsPtr &_msg);

      /// \brief Primary collision callback.
      /// \param[in] _data Pointer to user data.
      /// \param[in] _o1 First geom to check for collisions.
      /// \param[in] _o2 Second geom to check for collisions.
      private: static void CollisionCallback(void *_data, dGeomID _o1,
                                             dGeomID _o2);


      /// \brief Create a triangle mesh object collider.
      /// \param[in] _collision1 The first collision object.
      /// \param[in] _collision2 The second collision object.
      private: void AddTrimeshCollider(ODECollision *_collision1,
                                       ODECollision *_collision2);

      /// \brief Create a normal object collider.
      /// \param[in] _collision1 The first collision object.
      /// \param[in] _collision2 The second collision object.
      private: void AddCollider(ODECollision *_collision1,
                                ODECollision *_collision2);

      /// \brief Physics step function.
      public: int (*physicsStepFunc)(dxWorld*, dReal);

      /// \brief Current index into the contactFeedbacks buffer
      private: unsigned int jointFeedbackIndex;

      /// \brief Number of normal colliders.
      private: unsigned int collidersCount;

      /// \brief Number of triangle mesh colliders.
      private: unsigned int trimeshCollidersCount;

      /// \brief Maximum number of contact points per collision pair.
      private: unsigned int maxContacts;

      /// \internal
      /// \brief Private data pointer.
      private: ODEPhysicsPrivate *dataPtr;
    };
  }
}
#endif
