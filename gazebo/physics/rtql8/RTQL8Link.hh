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

#ifndef _RTQL8LINK_HH_
#define _RTQL8LINK_HH_

#include "gazebo/physics/Link.hh"

#include "gazebo/physics/rtql8/rtql8_inc.h"
#include "gazebo/physics/rtql8/RTQL8Types.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_rtql8 RTQL8 Physics
    /// \brief rtql8 physics engine wrapper
    /// \{

    /// \brief RTQL8 Link class
    class RTQL8Link : public Link
    {
      /// \brief Constructor
      public: explicit RTQL8Link(EntityPtr _parent);

      /// \brief Destructor
      public: virtual ~RTQL8Link();

      // Documentation inherited
      public: virtual void Load(sdf::ElementPtr _ptr);

      /// \brief Initialize the body
      public: virtual void Init();

      /// \brief Finalize the body
      public: virtual void Fini();

      /// \brief Update the body
      public: virtual void Update();
	  
	  // Documentation inherited
      public: virtual void OnPoseChange();

      // Documentation inherited
      public: virtual void SetEnabled(bool _enable) const;

      // Documentation inherited
      public: virtual bool GetEnabled() const;

      // Documentation inherited
      public: virtual void UpdateMass();

      // Documentation inherited
      public: virtual void UpdateSurface();

      // Documentation inherited
      public: virtual void SetLinearVel(const math::Vector3 &_vel);

      // Documentation inherited
      public: virtual void SetAngularVel(const math::Vector3 &_vel);

      // Documentation inherited
      public: virtual void SetForce(const math::Vector3 &_force);

      // Documentation inherited
      public: virtual void SetTorque(const math::Vector3 &_torque);

      // Documentation inherited
      public: virtual void AddForce(const math::Vector3 &_force);

      // Documentation inherited
      public: virtual void AddRelativeForce(const math::Vector3 &_force);

      // Documentation inherited
      public: virtual void AddForceAtWorldPosition(const math::Vector3 &_force,
                                                   const math::Vector3 &_pos);

      // Documentation inherited
      public: virtual void AddForceAtRelativePosition(
                  const math::Vector3 &_force,
                  const math::Vector3 &_relpos);

      // Documentation inherited
      public: virtual void AddTorque(const math::Vector3 &_torque);

      // Documentation inherited
      public: virtual void AddRelativeTorque(const math::Vector3 &_torque);

      // Documentation inherited
      public: virtual math::Vector3 GetWorldLinearVel() const;

      // Documentation inherited
      public: virtual math::Vector3 GetWorldAngularVel() const;

      // Documentation inherited
      public: virtual math::Vector3 GetWorldForce() const;

      // Documentation inherited
      public: virtual math::Vector3 GetWorldTorque() const;

      // Documentation inherited
      public: virtual void SetGravityMode(bool _mode);

      // Documentation inherited
      public: virtual bool GetGravityMode();

      // Documentation inherited
      public: void SetSelfCollide(bool _collide);

      // Documentation inherited
      public: virtual void SetLinearDamping(double _damping);

      // Documentation inherited
      public: virtual void SetAngularDamping(double _damping);

      // Documentation inherited
      public: virtual void SetKinematic(const bool &_state);

      // Documentation inherited
      public: virtual bool GetKinematic() const;

      // Documentation inherited
      public: virtual void SetAutoDisable(bool _disable);

      /// \brief
      public: rtql8::kinematics::BodyNode* GetBodyNode() const {return rtql8BodyNode;}

      /// \brief
      private: rtql8::kinematics::BodyNode* rtql8BodyNode;
	  
      /// \brief
      private: RTQL8PhysicsPtr rtql8Physics;

    };
    /// \}
  }
}
#endif
