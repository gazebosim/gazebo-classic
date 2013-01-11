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

#include "physics/simbody/simbody_inc.h"
#include "physics/PhysicsEngine.hh"
#include "physics/Collision.hh"
#include "physics/Shape.hh"

// #include <SimTKcommon.h>
#include <SimTKsimbody.h>
// #include <Simbody.h>
// using namespace SimTK;

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

      /// \brief Init the engine for threads.
      public: virtual void InitForThread();

      /// \brief Update the Simbody collision
      public: virtual void UpdateCollision();

      /// \brief Update the Simbody engine
      public: virtual void UpdatePhysics();

      /// \brief Finilize the Simbody engine
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

      private: SimTK::MultibodySystem *dynamicsWorld;

      private: common::Time lastUpdateTime;

      private: double stepTimeDouble;

      public: SimTK::MultibodySystem system;
      public: SimTK::SimbodyMatterSubsystem matter;
      public: SimTK::GeneralForceSubsystem forces;
      public: SimTK:: Integrator *integ;
    };

  /// \}
  }
}
#endif
