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
/* Desc: An ODE ball joint
 * Author: Nate Keonig
 * Date: 13 Oct 2009
 * SVN: $Id: BallJoint.hh 7039 2008-09-24 18:06:29Z natepak $
 */

#ifndef ODEBALLJOINT_HH
#define ODEBALLJOINT_HH

#include "BallJoint.hh"
#include "ODEJoint.hh"

namespace gazebo
{
  
  /// \addtogroup gazebo_physics_joints
  /// \{
  /** \defgroup gazebo_ball_joint Ball Joint
  
    \brief A ball joint
  
    \par Attributes
    - body1 (string)
      - Name of the first body to attach to the joint
    - body2 (string)
      - Name of the second body to attach to the joint
    - anchor (string)
      - Name of the body which will act as the anchor to the joint
    - erp (double)
      - Error reduction parameter. Default = 0.4
    - cfm (double)
      - Constraint force mixing. Default = 0.8
  
    \par Example
    \verbatim
    <joint:ball name="ball_joint>
      <body1>body1_name</body1>
      <body2>body2_name</body2>
      <anchor>anchor_body</anchor>
    </joint:ball>
    \endverbatim
  */
  /// \}
  /// \addtogroup gazebo_ball_joint
  /// \{
  
  /// \brief A ball joint
  class ODEBallJoint : public BallJoint<ODEJoint>
  {
    /// \brief Constructor
    public: ODEBallJoint( dWorldID worldId );
  
    /// \brief Destructor
    public: virtual ~ODEBallJoint();
  
    /// \brief Get joint's anchor point
    public: virtual Vector3 GetAnchor(int index) const;
  
    /// \brief Set joint's anchor point
    public: virtual void SetAnchor( int index, const Vector3 &anchor );

    /// \brief Get the axis of rotation
    public: virtual Vector3 GetAxis(int index) const {}

    /// \brief Set joint damping, not yet implemented
    public: virtual void SetDamping(int index, const double damping);

    /// \brief Set the velocity of an axis(index).
    public: virtual void SetVelocity(int index, double angle) {}

    /// \brief Get the rotation rate of an axis(index)
    public: virtual double GetVelocity(int index) const {}

    /// \brief Get the max allowed force of an axis(index).
    public: virtual double GetMaxForce(int index) {}

    /// \brief Set the max allowed force of an axis(index).
    public: virtual void SetMaxForce(int index, double t) {}

    /// \brief Get the angle of rotation of an axis(index)
    public: virtual Angle GetAngle(int index) const {}
 
  };
  
  /// \}
}

#endif
