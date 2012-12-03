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

      /// \brief Load the Bullet engine
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Initialize the Bullet engine
      public: virtual void Init();

      /// \brief Init the engine for threads.
      public: virtual void InitForThread();

      /// \brief Update the Bullet collision
      public: virtual void UpdateCollision();

      /// \brief Update the Bullet engine
      public: virtual void UpdatePhysics();

      /// \brief Finilize the Bullet engine
      public: virtual void Fini();

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

      /// \brief Convert an bullet mass to a gazebo Mass
      public: virtual void ConvertMass(InertialPtr _inertial,
                                       void *_engineMass);

      /// \brief Convert an gazebo Mass to a bullet Mass
      public: virtual void ConvertMass(void *_engineMass,
                                       InertialPtr _inertial);

      /// \brief Convert a bullet transform to a gazebo pose
      public: static math::Pose ConvertPose(const btTransform &_bt);

      /// \brief Convert a gazebo pose to a bullet transform
      public: static btTransform ConvertPose(const math::Pose &_pose);

      /// \brief Register a joint with the dynamics world
      public: btDynamicsWorld *GetDynamicsWorld() const
              {return this->dynamicsWorld;}

      /// \brief Set the gavity vector
      public: virtual void SetGravity(const gazebo::math::Vector3 &gravity);

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
