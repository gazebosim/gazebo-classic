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

#ifndef _RTQL8PHYSICS_HH
#define _RTQL8PHYSICS_HH

#include <string>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "physics/rtql8/rtql8_inc.h"

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
    /// \addtogroup gazebo_physics_rtql8 RTQL8 Physics
    /// \{

    /// \brief RTQL8 physics engine
    class RTQL8Physics : public PhysicsEngine
    {
      /// \brief Constructor
      public: RTQL8Physics(WorldPtr _world);
 
      /// \brief Destructor
      public: virtual ~RTQL8Physics();
       
      /// \brief Load the RTQL8 engine
      public: virtual void Load(sdf::ElementPtr _sdf);
 
      /// \brief Initialize the RTQL8 engine
      public: virtual void Init();

      /// \brief Finilize the RTQL8 engine.
      public: virtual void Fini();
       
      /// \brief Rest the RTQL8 engine.
      public: virtual void Reset();
 
      /// \brief Init the RTQL8 for threads.
      public: virtual void InitForThread();

      /// \brief Update the RTQL8 collision
      public: virtual void UpdateCollision();

      /// \brief Update the Bullet engine
      public: virtual void UpdatePhysics();

      /// \brief Set the simulation step time
      public: virtual void SetStepTime(double _value);

      /// \brief Get the simulation step time
      public: virtual double GetStepTime();

      /// \brief Create a new model (js: in test)
      //public: virtual ModelPtr CreateModel(ModelPtr _parent);
      
      /// \brief Create a new body
      public: virtual LinkPtr CreateLink(ModelPtr _parent);

      /// \brief Create a new collision
      public: virtual CollisionPtr CreateCollision(const std::string& _type,
						    LinkPtr _body);

      /// \brief Create a new joint
      public: virtual JointPtr CreateJoint(const std::string& _type,
					    ModelPtr _parent);
      
      /// \brief Create a new shape
      public: virtual ShapePtr CreateShape(const std::string& _shapeType,
					    CollisionPtr _collision);

      /// \brief Set the gavity vector
      public: virtual void SetGravity(const gazebo::math::Vector3& gravity);

      /// \brief Debug print out of the physic engine state.
      public: virtual void DebugPrint() const;

      ///// \brief 
      //private: common::Time lastUpdateTime;
 
      /// \brief Store the value of the stepTime parameter to improve efficiency
      private: double stepTimeDouble;
      
      /// \brief 
      private: simulation::World* rtql8World;
      
    };

  /// \}
  }
}
#endif
