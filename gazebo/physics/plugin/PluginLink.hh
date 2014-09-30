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
/* Desc: Plugin Link class
 * Author: Nate Koenig
 */

#ifndef _PluginLINK_HH_
#define _PluginLINK_HH_

#include "gazebo/physics/plugin/PluginTypes.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \brief Plugin Link class.
    class GAZEBO_VISIBLE PluginLink : public Link
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent model.
      public: explicit PluginLink(EntityPtr _parent);

      /// \brief Destructor.
      public: virtual ~PluginLink();

      // Documentation inherited
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited
      public: virtual void Init();

      // Documentation inherited
      public: virtual void Fini();

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
      public: virtual math::Vector3 GetWorldLinearVel(
          const math::Vector3 &_offset) const;

      // Documentation inherited
      public: virtual math::Vector3 GetWorldLinearVel(
                  const math::Vector3 &_offset,
                  const math::Quaternion &_q) const;

      // Documentation inherited
      public: virtual math::Vector3 GetWorldCoGLinearVel() const;

      // Documentation inherited
      public: virtual math::Vector3 GetWorldAngularVel() const;

      // Documentation inherited
      public: virtual math::Vector3 GetWorldForce() const;

      // Documentation inherited
      public: virtual math::Vector3 GetWorldTorque() const;

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

      /// \brief Callback when Plugin determines a body is disabled.
      /// \param[in] _id Id of the body.
      public: static void DisabledCallback(int _id);

      // Documentation inherited
      public: virtual void SetLinkStatic(bool _static);

      /// \brief Plugin link handle
      private: int linkId;

      /// \brief Pointer to the Plugin Physics engine
      private: PluginPhysicsPtr pluginPhysics;
    };
  }
}
#endif
