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
 * CVS: $Id: Hinge2Joint.hh,v 1.1.2.1 2006/12/16 22:41:15 natepak Exp $
 */

#ifndef HINGE2JOINT_HH
#define HINGE2JOINT_HH

#include "Vector3.hh"
#include "Joint.hh"

class Hinge2Joint : public Joint
{
  /// @brief Constructor
  public: Hinge2Joint(dWorldID worldId);

  // Destructor
  public: virtual ~Hinge2Joint(); 
 
  /// @brief Set the anchor point
  public: virtual void SetAnchor( const Vector3 &anchor );

  /// @brief Set the first axis of rotation
  public: void SetAxis1( const Vector3 &axis );

  /// @brief Set the second axis of rotation
  public: void SetAxis2( const Vector3 &axis );

  /// @brief Get the specified parameter
  public: virtual double GetParam( int parameter ) const;

  /// @brief Set _parameter with _value
  public: virtual void SetParam( int parameter, double value );

  /// @brief Get anchor point
  public: virtual Vector3 GetAnchor() const;

  /// @brief Get anchor point 2
  public: Vector3 GetAnchor2() const;

  /// @brief Get first axis of rotation
  public: Vector3 GetAxis1() const;

  /// @brief Get second axis of rotation
  public: Vector3 GetAxis2() const;

  /// @brief Get angle of rotation about first axis
  public: double GetAngle1() const;

  /// @brief Get rate of rotation about first axis
  public: double GetAngle1Rate() const;

  /// @brief Get rate of rotation about second axis
  public: double GetAngle2Rate() const;

  public: void SetTorque(double torque1, double torque2);
};

#endif

