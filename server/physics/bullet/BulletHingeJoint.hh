/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: A body that has a box shape
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 * CVS: $Id: BulletHingeJoint.hh 7039 2008-09-24 18:06:29Z natepak $
 */

#ifndef BULLETHINGEJOINT_HH
#define BULLETHINGEJOINT_HH

#include "Angle.hh"
#include "Vector3.hh"
#include "Param.hh"
#include "BulletJoint.hh"
#include "HingeJoint.hh"
#include "BulletPhysics.hh"

namespace gazebo
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
    protected: virtual void Load(XMLConfigNode *node);

    /// \brief Attach the two bodies with this joint
    public: virtual void Attach( Body *one, Body *two );

    /// \brief Get the anchor point
    public: virtual Vector3 GetAnchor(int index) const;
 
    /// \brief Set the anchor point
    public: virtual void SetAnchor(int index, const Vector3 &anchor);

    /// \brief Get the axis of rotation
    public: Vector3 GetAxis(int index) const;
 
    /// \brief Set the axis of rotation
    public: void SetAxis(int index, const Vector3 &axis);
 
    /// \brief Set joint damping, not yet implemented
    public: virtual void SetDamping(int index, const double damping);

    /// \brief Get the angle of rotation
    public: virtual Angle GetAngle(int index) const;

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
    public: virtual void SetHighStop(int index,Angle angle);

    /// \brief Set the low stop of an axis(index).
    public: virtual void SetLowStop(int index, Angle angle);
 
    /// \brief Get the high stop of an axis(index).
    public: virtual Angle GetHighStop(int index);

    /// \brief Get the low stop of an axis(index).
    public: virtual Angle GetLowStop(int index);
  };
  /// \}
  
}
#endif

