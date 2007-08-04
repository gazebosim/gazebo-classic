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
 * CVS: $Id: HingeJoint.hh,v 1.1.2.1 2006/12/16 22:41:15 natepak Exp $
 */

#ifndef HINGEJOINT_HH
#define HINGEJOINT_HH

#include "Vector3.hh"
#include "Joint.hh"

namespace gazebo
{

/// \addtogroup gazebo_physics_joints
/// \brief A single axis hinge joint
/// \{
/// \defgroup gazebo_hinge_joint Hinge Joint
/// \brief A single axis hinge joint
/// \{

/// \brief A single axis hinge joint
class HingeJoint : public Joint
{
  //!  Constructor
  public: HingeJoint(dWorldID worldId);

  //! Destructor
  public: virtual ~HingeJoint();

  //! Get the angle of rotation
  public: double GetAngle() const;

  //! Get the rotation rate
  public: double GetAngleRate() const;

  //! Get the specified parameter
  public: virtual double GetParam( int parameter ) const;

  //! Set the anchor point
  public: virtual void SetAnchor(const Vector3 &anchor);

  //! Set the axis of rotation
  public: void SetAxis(const Vector3 &axis);

  //! Get the anchor point
  public: virtual Vector3 GetAnchor() const;

  //! Get the axis of rotation
  public: Vector3 GetAxis() const;

  //! Set the parameter to value
  public: virtual void SetParam( int parameter, double value );

  //! Set the torque of a joint.
  public: void SetTorque(double torque);
};

/// \}
/// \}

}
#endif

