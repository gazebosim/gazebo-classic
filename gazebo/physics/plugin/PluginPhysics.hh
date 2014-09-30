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
#ifndef _PluginPHYSICS_HH_
#define _PluginPHYSICS_HH_

#include <tbb/spin_mutex.h>
#include <tbb/concurrent_vector.h>
#include <map>
#include <string>
#include <vector>
#include <utility>

#include <boost/thread/thread.hpp>

#include "gazebo/physics/plugin/PluginTypes.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/Contact.hh"
#include "gazebo/physics/Shape.hh"
#include "gazebo/gazebo_config.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \brief Data structure for contact feedbacks
    class GAZEBO_VISIBLE PluginJointFeedback
    {
      public: PluginJointFeedback() : contact(NULL), count(0) {}

      /// \brief Contact information.
      public: Contact *contact;

      /// \brief Number of elements in feedbacks array.
      public: int count;

      class JointFeedback
      {
        // feedback data structure
      };
      /// \brief Contact joint feedback information.
      public: JointFeedback feedbacks[MAX_CONTACT_JOINTS];
    };

    /// \brief Plugin physics engine.
    class GAZEBO_VISIBLE PluginPhysics : public PhysicsEngine
    {
      /// \enum PluginParam
      /// \brief Plugin Physics parameter types.
      public: enum PluginParam
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
        MIN_STEP_SIZE
      };

      /// \brief Constructor.
      /// \param[in] _world The World that uses this physics engine.
      public: PluginPhysics(WorldPtr _world);

      /// \brief Destructor.
      public: virtual ~PluginPhysics();

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
                      { return "plugin"; }

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
      public: virtual void SetMaxContacts(unsigned int max_contacts);

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

      /// \brief Get the world id.
      /// \return The world id.
      public: int GetWorldId();

      protected: virtual void OnRequest(ConstRequestPtr &_msg);

      protected: virtual void OnPhysicsMsg(ConstPhysicsPtr &_msg);

      /// \brief Top-level world for all bodies
      private: int worldId;

      /// \brief The type of the solver.
      private: std::string stepType;

      /// \brief Buffer of contact feedback information.
      private: std::vector<PluginJointFeedback*> jointFeedbacks;

      /// \brief Current index into the contactFeedbacks buffer
      private: unsigned int jointFeedbackIndex;

      /// \brief All the normal colliders.
      private: std::vector< std::pair<PluginCollision*, PluginCollision*> >
               colliders;

      /// \brief All the triangle mesh colliders.
      private: std::vector< std::pair<PluginCollision*, PluginCollision*> >
               trimeshColliders;

      /// \brief Number of normal colliders.
      private: unsigned int collidersCount;

      /// \brief Number of triangle mesh colliders.
      private: unsigned int trimeshCollidersCount;

      /// \brief Maximum number of contact points per collision pair.
      private: unsigned int maxContacts;
    };
  }
}
#endif
