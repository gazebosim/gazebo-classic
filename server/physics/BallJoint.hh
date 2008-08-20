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
 * CVS: $Id$
 */

#ifndef BALLJOINT_HH
#define BALLJOINT_HH

#include "Joint.hh"


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
class BallJoint : public Joint
{
  /// \brief Constructor
  public: BallJoint( dWorldID worldId );

  /// \brief Destructor
  public: virtual ~BallJoint();

  /// \brief Load the joint
  protected: void LoadChild(XMLConfigNode *node);

  /// \brief Save a joint to a stream in XML format
  protected: void SaveChild(std::string &prefix, std::ostream &stream);

  /// \brief Get joint's anchor point
  public: Vector3 GetAnchor() const;

  /// \brief Set joint's anchor point
  public: void SetAnchor( const Vector3 &anchor );

};

/// \}
}

#endif
