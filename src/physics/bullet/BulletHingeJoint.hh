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
/* Desc: A body that has a box shape
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 * CVS: $Id: BulletHingeJoint.hh 7039 2008-09-24 18:06:29Z natepak $
 */

#ifndef BULLETHINGEJOINT_HH
#define BULLETHINGEJOINT_HH

#include "common/Angle.hh"
#include "common/Vector3.hh"
#include "common/Param.hh"
#include "BulletJoint.hh"
#include "HingeJoint.hh"
#include "BulletPhysics.hh"

namespace gazebo
{
	namespace physics
{

  /// \addtogroup gazebo_physics_joints
  /// \{
  /** \defgroup gazebo_hinge_joint Hinge Joint
    
    \brief A two-axis hinge joint.
  
    \par Attributes
    - body1 (string)
      - Name of the first body to attach to the joint
    - body2 (string)
      - Name of the second body to attach to the joint
    - anchor (string)
      - Name of the body which will act as the anchor to the joint
    - axis (float, tuple)
      - Defines the axis of rotation for the first degree of freedom
      - Default: 0 0 1
    - lowStop (float, degrees)
      - The low stop angle for the first degree of freedom
      - Default: infinity
    - highStop (float, degrees)
      - The high stop angle for the first degree of freedom
      - Default: infinity
    - erp (double)
      - Error reduction parameter. 
      - Default = 0.4
    - cfm (double)
      - Constraint force mixing. 
      - Default = 0.8
  
    \par Example
    \verbatim
    <joint:hinge name="hinge_joint>
      <body1>body1_name</body1>
      <body2>body2_name</body2>
      <anchor>anchor_body</anchor>
      <axis>0 0 1</axis>
      <lowStop>0</lowStop>
      <highStop>30</highStop>
    </joint:hinge>
    \endverbatim
  */
  /// \}
  
  /// \addtogroup gazebo_hinge_joint
  /// \{
  
  ///\brief A single axis hinge joint
  class BulletHingeJoint : public HingeJoint<BulletJoint>
  {
    ///  Constructor
    public: BulletHingeJoint(btDynamicsWorld *world);
  
    /// Destructor
    public: virtual ~BulletHingeJoint();
  
    /// \brief Load joint
    protected: virtual void Load(common::XMLConfigNode *node);

    /// \brief Attach the two bodies with this joint
    public: virtual void Attach( Body *one, Body *two );

    /// \brief Get the anchor point
    public: virtual common::Vector3 GetAnchor(int index) const;
 
    /// \brief Set the anchor point
    public: virtual void SetAnchor(int index, const common::Vector3 &anchor);

    /// \brief Get the axis of rotation
    public: common::Vector3 GetAxis(int index) const;
 
    /// \brief Set the axis of rotation
    public: void SetAxis(int index, const common::Vector3 &axis);
 
    /// \brief Set joint damping, not yet implemented
    public: virtual void SetDamping(int index, const double damping);

    /// \brief Get the angle of rotation
    public: virtual common::Angle GetAngle(int index) const;

     /// \brief Set the velocity of an axis(index).
    public: virtual void SetVelocity(int index, double angle);
 
    /// \brief Get the rotation rate
    public: virtual double GetVelocity(int index) const;

    /// \brief Set the max allowed force of an axis(index).
    public: virtual void SetMaxForce(int index, double t);

    /// \brief Get the max allowed force of an axis(index).
    public: virtual double GetMaxForce(int index);

    /// \brief Set the torque of a joint.
    public: void SetForce(int index, double torque);

    /// \brief Get the torque of a joint.
    public: virtual double GetForce(int index);

    /// \brief Set the high stop of an axis(index).
    public: virtual void SetHighStop(int index,common::Angle angle);

    /// \brief Set the low stop of an axis(index).
    public: virtual void SetLowStop(int index, common::Angle angle);
 
    /// \brief Get the high stop of an axis(index).
    public: virtual common::Angle GetHighStop(int index);

    /// \brief Get the low stop of an axis(index).
    public: virtual common::Angle GetLowStop(int index);
  };
  /// \}
  
}
}
}
#endif

