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
/* Desc: Bullet motion state class.
 * Author: Nate Koenig 
 * Date: 25 May 2009
 */

#ifndef BULLETMOTIONSTATE
#define BULLETMOTIONSTATE

#include <btBulletDynamicsCommon.h>

#include "math/Pose.hh"

namespace gazebo
{
	namespace physics
{
  class Visual;
  class Body;

  class BulletMotionState : public btMotionState
  {
    /// \brief Constructor
    public: BulletMotionState(Body *body);

    /// \brief Constructor
    //public: BulletMotionState(const math::Pose &initPose);

    /// \brief Destructor
    public: virtual ~BulletMotionState();

    /// \brief Set the visual
    public: void SetVisual(Visual *vis);

    /// \brief Get the pose
    public: math::Pose GetWorldPose() const;

    /// \brief Set the position of the body
    /// \param pos math::Vector position
    public: virtual void SetWorldPosition(const math::Vector3 &pos);

    /// \brief Set the rotation of the body
    /// \param rot Quaternion rotation
    public: virtual void SetWorldRotation(const common::Quatern &rot);

    /// \brief Set the pose
    public: void SetWorldPose(const math::Pose &pose);

    /// \brief Set the center of mass offset
    public: void SetCoMOffset( const math::Pose &com );

    /// \brief Get the world transform
    public: virtual void getWorldTransform(btTransform &worldTrans) const;

    /// \brief Set the world transform
    public: virtual void setWorldTransform(const btTransform &worldTrans);

    private: Visual *visual;
    private: math::Pose worldPose;
    private: math::Pose comOffset;
    private: Body *body;
  };
}
}
}
#endif
