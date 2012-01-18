/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
 * SVN: $Id: BulletPhysics.hh 7706 2009-05-21 16:46:22Z natepak $
 */

#ifndef BULLETPHYSICS_HH
#define BULLETPHYSICS_HH

#include <btBulletDynamicsCommon.h>
#include <btBulletCollisionCommon.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "PhysicsEngine.hh"
#include "physics/Collision.hh"
#include "Shape.hh"

namespace gazebo
{
  namespace physics
  {
    class Entity;
    class XMLConfigNode;
    class Mass;
  
    /// \addtogroup gazebo_physics_engine
    /// \{
    /** \defgroup bulletphysicsengine Bullet Physics Engine
  
      The \c param:physics tag is used to specify certain parameters for the
      Bullet phyics engine. The following parameters are in addition to those
      provided by the PhysicsEngine base class.
  
    \par Attributes
    
    - cfm (float)
      - Global constraint force mixing
      - Default: 10e-5
      - Range:  10e-10 to 1.0
      - Recommended value: 10e-5
    - erp (float)
      - Global error reduction parameter
      - Default: 0.2
      - Range: 0 to 1.0
      - Recommended Range: 0.1 to 0.8
    - stepcommon::Time (float)
      - Time, in seconds, that elapse for each iteration of the physics engine
      - Default: 0.025
    -gravity (float float float)
      - Gravity vector.
      - Default: 0 0 -9.8
    
    \verbatim
    <physics:bullet>
      <stepTime>0.03</stepTime>
      <gravity>0 0 -9.8</gravity>
      <cfm>10e-5</cfm>
      <erp>0.2</erp>
    </physcis:bullet>
    \endverbatim
    
    \{
    */
  
  /// \brief Bullet physics engine
  class BulletPhysics : public PhysicsEngine
  {
    /// \brief Constructor
    public: BulletPhysics(World *world);
  
    /// \brief Destructor
    public: virtual ~BulletPhysics();
  
    /// \brief Load the Bullet engine
    public: virtual void Load(common::XMLConfigNode *node);
  
    /// \brief Saves to XMLFile
    public: void Save(std::string &prefix, std::ostream &stream);
  
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
  
    /// \brief Add an entity to the world
    public: void AddEntity(Entity *entity);
  
    /// \brief Remove an entity from the physics engine
    public: virtual void RemoveEntity(Entity *entity);
  
    /// \brief Create a new body
    public: virtual Link *CreateLink(Entity *parent);
  
    /// \brief Create a new collision
    public: virtual Collision *CreateCollision(std::string type, Link *body);
  
    /// \brief Create a new joint
    public: virtual Joint *CreateJoint(std::string type);
  
    /// \brief Create a physics based ray sensor
    // public: virtual PhysicsRaySensor *CreateRaySensor(Link *body);
  
    /// \brief Convert an bullet mass to a gazebo Mass
    public: virtual void ConvertMass(Mass *mass, void *engineMass);
  
    /// \brief Convert an gazebo Mass to a bullet Mass
    public: virtual void ConvertMass(void *engineMass, const Mass &mass);
  
    /// \brief Convert a bullet transform to a gazebo pose
    public: static math::Pose ConvertPose(btTransform bt);
  
    /// \brief Convert a gazebo pose to a bullet transform
    public: static btTransform ConvertPose(const math::Pose pose);
  
    /// \brief Register a joint with the dynamics world
    public: btDynamicsWorld *GetDynamicsWorld() const
            {return this->dynamicsWorld;}
    /// \brief Set the gavity vector
    public: virtual void SetGravity(const gazebo::math::Vector3 &gravity);
  
    // private: btAxisSweep3 *broadPhase;
    private: btBroadphaseInterface *broadPhase;
    private: btDefaultCollisionConfiguration *collisionConfig;
    private: btCollisionDispatcher *dispatcher;
    private: btSequentialImpulseConstraintSolver *solver;
    private: btDiscreteDynamicsWorld *dynamicsWorld;
  
    private: common::Time lastUpdateTime;
  };
  
  /** \}*/
  /// \}
  }

}
#endif


