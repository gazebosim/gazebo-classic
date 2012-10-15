/*
 * Copyright 2011 Nate Koenig
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
/* Desc: The ODE physics engine wrapper
 * Author: Nate Koenig
 * Date: 11 June 2007
 */

#ifndef ODEPHYSICS_HH
#define ODEPHYSICS_HH

#include <tbb/spin_mutex.h>
#include <tbb/concurrent_vector.h>
#include <map>
#include <string>
#include <vector>
#include <utility>

#include <boost/thread/thread.hpp>

#include "physics/ode/ode_inc.h"

#include "physics/ode/ODETypes.hh"

#include "physics/PhysicsEngine.hh"
#include "physics/Contact.hh"
#include "physics/Shape.hh"
#include "gazebo_config.h"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_ode ODE Physics
    /// \{

    /// \brief data structure for contact feedbacks
    class ContactFeedback
    {
      public: Contact contact;
      public: dJointFeedback feedbacks[MAX_CONTACT_JOINTS];
      public: int feedbackCount;
    };

    /// \brief ODE physics engine
    class ODEPhysics : public PhysicsEngine
    {
      /// \brief Constructor
      public: ODEPhysics(WorldPtr world);

      /// \brief Destructor
      public: virtual ~ODEPhysics();

      /// \brief Load the ODE engine
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Initialize the ODE engine
      public: virtual void Init();

      /// \brief Empties dynamically created contact joints
      public: void Reset();

      /// \brief Init the engine for threads.
      public: virtual void InitForThread();

      /// \brief Update the ODE collision
      public: virtual void UpdateCollision();

      /// \brief Update the ODE engine
      public: virtual void UpdatePhysics();

      /// \brief Finilize the ODE engine
      public: virtual void Fini();

      /// \brief Set the step time
      public: void SetStepTime(double _value);

      /// \brief Get the simulation step time
      public: virtual double GetStepTime();

      /// \brief Create a new body
      public: virtual LinkPtr CreateLink(ModelPtr _parent);

      /// \brief Create a collision
      public: virtual CollisionPtr CreateCollision(
                  const std::string &_shapeType, LinkPtr _parent);

      /// \brief Create a collision physics::Shape object given its type
      public: virtual ShapePtr CreateShape(const std::string &_shapeType,
                                           CollisionPtr _collision);

      /// \brief Create a new joint
      public: virtual JointPtr CreateJoint(const std::string &type,
                                           ModelPtr _parent);

      /// \brief Return the space id
      public: dSpaceID GetSpaceId() const;

      /// \brief Get the world id
      public: dWorldID GetWorldId();

      /// \brief Convert an odeMass to Mass
      public: static void ConvertMass(InertialPtr _interial, void *odeMass);

      /// \brief Convert an odeMass to Mass
      public: static void ConvertMass(void *odeMass, InertialPtr _inertial);

      /// \brief Get the step type
      public: virtual std::string GetStepType() const;

      /// \brief Set the step type
      public: virtual void SetStepType(const std::string &_type);

      /// \brief Set the gavity vector
      public: virtual void SetGravity(const gazebo::math::Vector3 &gravity);

      /// \brief Get gravity vector
      public: math::Vector3 GetGravity() const;

      /// \brief access functions to set ODE parameters
      public: void SetWorldCFM(double cfm);
      /// \brief access functions to set ODE parameters
      public: void SetWorldERP(double erp);
      /// \brief access functions to set ODE parameters
      public: void SetSORPGSPreconIters(unsigned int iters);
      /// \brief access functions to set ODE parameters
      public: void SetSORPGSIters(unsigned int iters);
      /// \brief access functions to set ODE parameters
      public: void SetSORPGSW(double w);
      /// \brief access functions to set ODE parameters
      public: void SetContactMaxCorrectingVel(double vel);
      /// \brief access functions to set ODE parameters
      public: void SetContactSurfaceLayer(double layer_depth);
      /// \brief access functions to set ODE parameters
      public: void SetMaxContacts(unsigned int max_contacts);

      /// \brief access functions to set ODE parameters
      public: double GetWorldCFM();
      /// \brief access functions to set ODE parameters
      public: double GetWorldERP();
      /// \brief access functions to set ODE parameters
      public: int GetSORPGSPreconIters();
      /// \brief access functions to set ODE parameters
      public: int GetSORPGSIters();
      /// \brief access functions to set ODE parameters
      public: double GetSORPGSW();
      /// \brief access functions to set ODE parameters
      public: double GetContactMaxCorrectingVel();
      /// \brief access functions to set ODE parameters
      public: double GetContactSurfaceLayer();
      /// \brief access functions to set ODE parameters
      public: int GetMaxContacts();

      /// \brief Not yet implemented
      public: void CreateContact(ODECollision *collision1,
                                 ODECollision *collision2);

      /// \brief Do collision detection
      private: static void CollisionCallback(void *data,
                                             dGeomID o1, dGeomID o2);

      /// \brief Collide two collisions
      public: void Collide(ODECollision *collision1, ODECollision *collision2,
                           dContactGeom *contactCollisions);

      /// \brief process contact feedbacks into physics::ContactFeedback
      public: void ProcessContactFeedback(ContactFeedback* feedback,
                                          msgs::Contact *_msg);

      public: virtual void DebugPrint() const;

      /// \brief act on a msgs::Request for physics
      protected: virtual void OnRequest(
                   ConstRequestPtr &/*_msg*/);

      /// \brief act on a msgs::Physics message request
      protected: virtual void OnPhysicsMsg(
                   ConstPhysicsPtr &/*_msg*/);

      private: void  AddTrimeshCollider(ODECollision *_collision1,
                                         ODECollision *_collision2);


      private: void  AddCollider(ODECollision *_collision1,
                                  ODECollision *_collision2);

      /// \brief Top-level world for all bodies
      private: dWorldID worldId;

      /// \brief Top-level space for all sub-spaces/collisions
      private: dSpaceID spaceId;

      /// \brief Collision attributes
      private: dJointGroupID contactGroup;

      /// Store the value of the stepTime parameter in doubl form. To improve
      /// efficiency
      private: double stepTimeDouble;
      private: std::string stepType;

      private: std::vector<ContactFeedback*> contactFeedbacks;
      private: unsigned int contactFeedbackIndex;

      private: std::map<std::string, dSpaceID> spaces;

      private: std::vector< std::pair<ODECollision*, ODECollision*> > colliders;
      private: std::vector< std::pair<ODECollision*, ODECollision*> >
               trimeshColliders;

      private: unsigned int collidersCount, trimeshCollidersCount;

      private: tbb::spin_mutex collideMutex;

      private: dContactGeom contactCollisions[MAX_COLLIDE_RETURNS];
      private: int (*physicsStepFunc)(dxWorld*, dReal);

      private: int indices[MAX_CONTACT_JOINTS];
    };
    typedef boost::shared_ptr<ODEPhysics> ODEPhysicsPtr;

  /// \}
  }
}
#endif
