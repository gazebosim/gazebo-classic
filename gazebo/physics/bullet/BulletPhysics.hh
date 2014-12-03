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
/* Desc: The Bullet physics engine wrapper
 * Author: Nate Koenig
 * Date: 11 June 2009
 */

#ifndef BULLETPHYSICS_HH
#define BULLETPHYSICS_HH
#include <string>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "gazebo/physics/bullet/bullet_inc.h"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/Shape.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    class Entity;
    class XMLConfigNode;
    class Mass;

    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_bullet Bullet Physics
    /// \{

    /// \brief Bullet physics engine
    class GZ_PHYSICS_VISIBLE BulletPhysics : public PhysicsEngine
    {
      /// \enum BulletParam
      /// \brief Bullet physics parameter types.
      public: enum BulletParam
      {
        /// \brief Solve type
        SOLVER_TYPE,

        /// \brief Constraint force mixing
        GLOBAL_CFM,

        /// \brief Error reduction parameter
        GLOBAL_ERP,

        /// \brief Number of iterations
        PGS_ITERS,

        /// \brief SOR over-relaxation parameter
        SOR,

        /// \brief Surface layer depth
        CONTACT_SURFACE_LAYER,

        /// \brief Maximum number of contacts
        MAX_CONTACTS,

        /// \brief Minimum step size
        MIN_STEP_SIZE
      };

      /// \brief Constructor
      public: BulletPhysics(WorldPtr _world);

      /// \brief Destructor
      public: virtual ~BulletPhysics();

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
                      { return "bullet"; }

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

      /// \brief Create a physics based ray sensor
      // public: virtual PhysicsRaySensor *CreateRaySensor(Link *body);

      // Documentation inherited
      protected: virtual void OnRequest(ConstRequestPtr &_msg);

      // Documentation inherited
      protected: virtual void OnPhysicsMsg(ConstPhysicsPtr &_msg);

      /// \brief Convert a bullet mass to a gazebo Mass
      public: virtual void ConvertMass(InertialPtr _inertial,
                                       void *_engineMass);

      /// \brief Convert a gazebo Mass to a bullet Mass
      public: virtual void ConvertMass(void *_engineMass,
                                       InertialPtr _inertial);

      // Documentation inherited
      public: virtual void SetGravity(const gazebo::math::Vector3 &_gravity);

      // Documentation inherited
      public: virtual void SetWorldCFM(double _cfm);

      // Documentation inherited
      public: virtual double GetWorldCFM();

      // Documentation inherited
      public: virtual void SetSeed(uint32_t _seed);

      /// \brief Register a joint with the dynamics world
      public: btDynamicsWorld *GetDynamicsWorld() const
              {return this->dynamicsWorld;}

      public: virtual void DebugPrint() const;

      /// Documentation inherited
      public: virtual bool SetParam(const std::string &_key,
                  const boost::any &_value);

      /// Documentation inherited
      public: virtual boost::any GetParam(const std::string &_key) const;

      // Documentation inherited
      public: virtual void SetSORPGSIters(unsigned int iters);

      private: btBroadphaseInterface *broadPhase;
      private: btDefaultCollisionConfiguration *collisionConfig;
      private: btCollisionDispatcher *dispatcher;
      private: btSequentialImpulseConstraintSolver *solver;
      private: btDiscreteDynamicsWorld *dynamicsWorld;

      private: common::Time lastUpdateTime;

      /// \brief The type of the solver.
      private: std::string solverType;
    };

  /// \}
  }
}
#endif
