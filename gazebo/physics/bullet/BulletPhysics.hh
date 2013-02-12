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
/* Desc: The Bullet physics engine wrapper
 * Author: Nate Koenig
 * Date: 11 June 2009
 */

#ifndef BULLETPHYSICS_HH
#define BULLETPHYSICS_HH
#include <string>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "physics/bullet/bullet_inc.h"
#include "physics/PhysicsEngine.hh"
#include "physics/Collision.hh"
#include "physics/Shape.hh"

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
    class BulletPhysics : public PhysicsEngine
    {
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
      public: virtual void SetStepTime(double _value);

      // Documentation inherited
      public: virtual double GetStepTime();

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

      private: btBroadphaseInterface *broadPhase;
      private: btDefaultCollisionConfiguration *collisionConfig;
      private: btCollisionDispatcher *dispatcher;
      private: btSequentialImpulseConstraintSolver *solver;
      private: btDiscreteDynamicsWorld *dynamicsWorld;

      private: common::Time lastUpdateTime;

      private: double stepTimeDouble;
    };

  /// \}
  }
}
#endif
