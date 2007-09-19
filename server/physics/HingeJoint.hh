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
 * CVS: $Id$
 */

#ifndef HINGEJOINT_HH
#define HINGEJOINT_HH

#include "Vector3.hh"
#include "Joint.hh"

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
class HingeJoint : public Joint
{
  ///  Constructor
  public: HingeJoint(dWorldID worldId);

  /// Destructor
  public: virtual ~HingeJoint();

  /// \brief Load joint
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// Get the angle of rotation
  public: double GetAngle() const;

  /// Get the rotation rate
  public: double GetAngleRate() const;

  /// Get the specified parameter
  public: virtual double GetParam( int parameter ) const;

  /// Set the anchor point
  public: virtual void SetAnchor(const Vector3 &anchor);

  /// Set the axis of rotation
  public: void SetAxis(const Vector3 &axis);

  /// Get the anchor point
  public: virtual Vector3 GetAnchor() const;

  /// Get the axis of rotation
  public: Vector3 GetAxis() const;

  /// Set the parameter to value
  public: virtual void SetParam( int parameter, double value);

  /// Set the torque of a joint.
  public: void SetTorque(double torque);
};
/// \}

}
#endif

