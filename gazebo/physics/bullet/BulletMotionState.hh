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
/* Desc: Bullet motion state class.
 * Author: Nate Koenig
 * Date: 25 May 2009
 */

#ifndef _BULLETMOTIONSTATE_HH_
#define _BULLETMOTIONSTATE_HH_

#include "physics/bullet/bullet_inc.h"
#include "math/MathTypes.hh"
#include "physics/PhysicsTypes.hh"
#include "math/Pose.hh"

namespace gazebo
{
  namespace physics
  {
    class Link;

    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_bullet Bullet Physics
    /// \{

    /// \brief Bullet btMotionState encapsulation
    class BulletMotionState : public btMotionState
    {
      /// \brief Constructor
      public: BulletMotionState(LinkPtr _link);

      /// \brief Constructor
      // public: BulletMotionState(const math::Pose &initPose);

      /// \brief Destructor
      public: virtual ~BulletMotionState();

      /// \brief Get the pose
      public: math::Pose GetWorldPose() const;

      /// \brief Set the position of the body
      /// \param pos math::Vector position
      public: virtual void SetWorldPosition(const math::Vector3 &_pos);

      /// \brief Set the rotation of the body
      /// \param rot Quaternion rotation
      public: virtual void SetWorldRotation(const math::Quaternion &_rot);

      /// \brief Set the pose
      public: void SetWorldPose(const math::Pose &_pose);

      /// \brief Set the center of mass offset
      public: void SetCoG(const math::Vector3 &_cog);

      /// \brief Get the world transform
      public: virtual void getWorldTransform(btTransform &_worldTrans) const;

      /// \brief Set the world transform
      public: virtual void setWorldTransform(const btTransform &_worldTrans);

      private: math::Pose worldPose;
      private: math::Vector3 cog;
      private: LinkPtr link;
    };
    /// \}
  }
}
#endif
