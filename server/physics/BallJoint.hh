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
/* Desc: A ball joint
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 * CVS: $Id: BallJoint.hh,v 1.1.2.1 2006/12/16 22:41:15 natepak Exp $
 */

#ifndef BALLJOINT_HH
#define BALLJOINT_HH

#include "Joint.hh"


namespace gazebo
{

/// \addtogroup gazebo_physics_joints
/// \brief A ball joint
/// \{
/// \defgroup gazebo_ball_joint Ball Joint
/// \brief A ball joint
/// \{

/// \brief A ball joint
class BallJoint : public Joint
{
  /// \brief Constructor
  public: BallJoint( dWorldID worldId );

  /// \brief Destructor
  public: virtual ~BallJoint();

  /// \brief Get joint's anchor point
  public: Vector3 GetAnchor() const;

  /// \brief Set joint's anchor point
  public: void SetAnchor( const Vector3 &anchor );

};

/// \}
/// \}
}

#endif
