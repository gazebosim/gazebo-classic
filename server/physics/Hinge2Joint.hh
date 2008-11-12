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
/* Desc: A hinge joint with 2 degrees of freedom
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 * CVS: $Id$
 */

#ifndef HINGE2JOINT_HH
#define HINGE2JOINT_HH

#include "Param.hh"
#include "Angle.hh"
#include "Vector3.hh"
#include "Joint.hh"

namespace gazebo
{

/// \addtogroup gazebo_physics_joints
/// \{
/** \defgroup gazebo_hinge2_joint Hinge 2 Joint
 
  \brief A two-axis hinge joint.

  \par Attributes
  - body1 (string)
    - Name of the first body to attach to the joint
  - body2 (string)
    - Name of the second body to attach to the joint
  - anchor (string)
    - Name of the body which will act as the anchor to the joint
  - axis1 (float, tuple)
    - Defines the axis of rotation for the first degree of freedom
    - Default: 0 0 1
  - axis2 (float, tuple)
    - Defines the axis of rotation for the second degree of freedom
    - Default: 0 0 1
  - lowStop1 (float, degrees)
    - The low stop angle for the first degree of freedom
    - Default: infinity
  - highStop1 (float, degrees)
    - The high stop angle for the first degree of freedom
    - Default: infinity
  - lowStop2 (float, degrees)
    - The low stop angle for the second degree of freedom
    - Default: infinity
  - highStop2 (float, degrees)
    - The high stop angle for the second degree of freedom
    - Default: infinity
  - erp (double)
    - Error reduction parameter. 
    - Default = 0.4
  - cfm (double)
    - Constraint force mixing. 
    - Default = 0.8


  \par Example
  \verbatim
  <joint:hinge2 name="hinge2_joint>
    <body1>body1_name</body1>
    <body2>body2_name</body2>
    <anchor>anchor_body</anchor>
    <axis1>0 0 1</axis1>
    <axis2>0 1 0</axis2>
    <lowStop1>0</lowStop1>
    <highStop1>30</highStop1>
    <lowStop2>0</lowStop2>
    <highStop2>30</highStop2>
  </joint:hinge2>
  \endverbatim
*/
/// \}
/// \addtogroup gazebo_hinge2_joint
/// \{

/// \brief A two axis hinge joint
class Hinge2Joint : public Joint
{
  /// \brief Constructor
  public: Hinge2Joint(dWorldID worldId);

  /// \brief Destructor
  public: virtual ~Hinge2Joint(); 

  /// \brief Load the joint
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// \brief Save a joint to a stream in XML format
  protected: virtual void SaveChild(std::string &prefix, std::ostream &stream);
 
  /// \brief Set the anchor point
  public: virtual void SetAnchor( const Vector3 &anchor );

  /// \brief Set the first axis of rotation
  public: void SetAxis1( const Vector3 &axis );

  /// \brief Set the second axis of rotation
  public: void SetAxis2( const Vector3 &axis );

  /// \brief Get the specified parameter
  public: virtual double GetParam( int parameter ) const;

  /// \brief Set _parameter with _value
  public: virtual void SetParam( int parameter, double value );

  /// \brief Get anchor point
  public: virtual Vector3 GetAnchor() const;

  /// \brief Get anchor point 2
  public: Vector3 GetAnchor2() const;

  /// \brief Get first axis of rotation
  public: Vector3 GetAxis1() const;

  /// \brief Get second axis of rotation
  public: Vector3 GetAxis2() const;

  /// \brief Get angle of rotation about first axis
  public: double GetAngle1() const;

  /// \brief Get rate of rotation about first axis
  public: double GetAngle1Rate() const;

  /// \brief Get rate of rotation about second axis
  public: double GetAngle2Rate() const;

  /// \brief Set the torque
  public: void SetTorque(double torque1, double torque2);

  private: ParamT<Vector3> *axis1P;
  private: ParamT<Angle> *loStop1P;
  private: ParamT<Angle> *hiStop1P; 

  private: ParamT<Vector3> *axis2P;
  private: ParamT<Angle> *loStop2P;
  private: ParamT<Angle> *hiStop2P; 

  private: ParamT<double> *suspensionCfmP;
};

/// \}
}
#endif

