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
/* Desc: A ball joint
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 * CVS: $Id: BulletBallJoint.hh 7039 2008-09-24 18:06:29Z natepak $
 */

#ifndef BULLETBALLJOINT_HH
#define BULLETBALLJOINT_HH

#include "BallJoint.hh"
#include "BulletJoint.hh"
#include "BulletPhysics.hh"

namespace gazebo
{
  namespace physics
  {
    /// \brief A ball joint
    class BulletBallJoint : public BallJoint<BulletJoint>
    {
      /// \brief Constructor
      public: BulletBallJoint(btDynamicsWorld *world);
  
      /// \brief Destructor
      public: virtual ~BulletBallJoint();
  
      /// \brief Get joint's anchor point
      public: math::Vector3 GetAnchor(int index) const;
  
      /// \brief Set joint's anchor point
      public: void SetAnchor(int index, const math::Vector3 &anchor);
  
      /// \brief Set joint damping, not yet implemented
      public: virtual void SetDamping(int index, const double damping);
  
      /// \brief Attach the two bodies with this joint
      public: void Attach(Link *one, Link *two);
  
      /// \brief Get the axis of rotation
      public: virtual math::Vector3 GetAxis(int index) const {}
      /// \brief Set the velocity of an axis(index).
      public: virtual void SetVelocity(int index, double angle) {}
      /// \brief Get the rotation rate of an axis(index)
      public: virtual double GetVelocity(int index) const {}
      /// \brief Get the max allowed force of an axis(index).
      public: virtual double GetMaxForce(int index) {}
      /// \brief Set the max allowed force of an axis(index).
      public: virtual void SetMaxForce(int index, double t) {}
      /// \brief Get the angle of rotation of an axis(index)
      public: virtual math::Angle GetAngle(int index) const {}
    };
  
  }
}
#endif

