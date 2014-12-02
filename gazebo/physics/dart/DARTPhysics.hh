/*
 * Copyright 2014 Open Source Robotics Foundation
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

#ifndef _GAZEBO_DARTPHYSICS_HH_
#define _GAZEBO_DARTPHYSICS_HH_

#include <string>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/Shape.hh"

#include "gazebo/physics/dart/dart_inc.h"
#include "gazebo/physics/dart/DARTTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_dart DART Physics
    /// \{

    /// \brief DART physics engine
    class GZ_PHYSICS_VISIBLE DARTPhysics : public PhysicsEngine
    {
      /// \enum DARTParam
      /// \brief DART physics parameter types.
      public: enum DARTParam
      {
        // /// \brief Solve type
        // SOLVER_TYPE,

        // /// \brief Constraint force mixing
        // GLOBAL_CFM,

        // /// \brief Error reduction parameter
        // GLOBAL_ERP,

        // /// \brief Number of iterations
        // PGS_ITERS,

        // /// \brief SOR over-relaxation parameter
        // SOR,

        // /// \brief Surface layer depth
        // CONTACT_SURFACE_LAYER,

        /// \brief Maximum number of contacts
        MAX_CONTACTS,

        /// \brief Minimum step size
        MIN_STEP_SIZE
      };

      /// \brief Constructor
      public: DARTPhysics(WorldPtr _world);

      /// \brief Destructor
      public: virtual ~DARTPhysics();

      // Documentation inherited
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited
      public: virtual void Init();

      // Documentation inherited
      public: virtual void Fini();

      // Documentation inherited
      public: virtual void Reset();

      // Documentation inherited
      public: virtual void InitForThread();

      // Documentation inherited
      public: virtual void UpdateCollision();

      // Documentation inherited
      public: virtual void UpdatePhysics();

      // Documentation inherited
      public: virtual std::string GetType() const;

      // Documentation inherited
      public: virtual void SetSeed(uint32_t _seed);

      // Documentation inherited
      public: virtual ModelPtr CreateModel(BasePtr _parent);

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

      // Documentation inherited
      public: virtual void SetGravity(const gazebo::math::Vector3 &_gravity);

      // Documentation inherited
      public: virtual void DebugPrint() const;

      // Documentation inherited
      public: virtual boost::any GetParam(const std::string &_key) const;

      // Documentation inherited
      public: virtual bool SetParam(const std::string &_key,
                  const boost::any &_value);

      /// \brief Get pointer to DART World associated with this DART Physics.
      /// \return The pointer to DART World.
      public: dart::simulation::World *GetDARTWorld();

      // Documentation inherited
      protected: virtual void OnRequest(ConstRequestPtr &_msg);

      // Documentation inherited
      protected: virtual void OnPhysicsMsg(ConstPhysicsPtr &_msg);

      /// \brief Find DART Link corresponding to DART BodyNode.
      /// \param[in] _dtBodyNode The DART BodyNode.
      /// \return Pointer to the DART Link.
      private: DARTLinkPtr FindDARTLink(
          const dart::dynamics::BodyNode *_dtBodyNode);

      /// \brief Pointer to DART World associated with this DART Physics.
      private: dart::simulation::World *dtWorld;
    };

  /// \}
  }
}
#endif
