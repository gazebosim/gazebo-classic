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
/* Desc: External interfaces for Gazebo
 * Author: Nate Koenig
 * Date: 03 Apr 2007
 * SVN: $Id$
 */
/// @addtogroup gazebocore
/// @{

#ifndef QUATERN_HH
#define QUATERN_HH

#include <iostream>
#include "Vector3.hh"

class Quatern;
std::ostream &operator<<(std::ostream &out, const Quatern &);

/// A quaternion class
class Quatern
{
  /// Default Constructor
  public: Quatern();

  /// Constructor
  /// \param u U param
  /// \param x X param
  /// \param y Y param
  /// \param z Z param
  public: Quatern( const double &u, const double &x, const double &y, const double &z);

  /// Copy constructor
  /// \param qt Quatern to copy
  public: Quatern( const Quatern &qt );

  /// Destructor
  public: ~Quatern();

  /// Equal operator
  /// \param qt Quatern to copy
  public: const Quatern &operator=(const Quatern &qt);

  /// Invert the quaternion
  public: void Invert();

  /// Get the inverse of this quaternion
  /// \return Inverse quarenion
  public: Quatern GetInverse() const;

  /// Set the quatern to the identity
  public: void SetToIdentity();

  /// Normalize the quaternion
  public: void Normalize();

  /// Set the quaternion from an axis and angle
  /// \param x X axis 
  /// \param y Y axis
  /// \param z Z axis
  /// \param a Angle in radians
  public: void SetFromAxis(double x, double y, double z, double a);
         
  /// Set the quaternion from Euler angles
  /// \param roll Roll in radians
  /// \param pitch Pitch in radians
  /// \param yaw Yaw in radians
  public: void SetFromEuler(double roll, double pitch, double yaw);

  /// Return the rotation in Euler angles
  /// \return This quaternion as an Euler vector
  public: Vector3 GetAsEuler();

  /// Return rotation as axis and angle (x, y, y, rotation)
  /// \return This quaternion as an axis-angle
  public: Quatern GetAsAxis();

  /// Scale a Quaternion
  /// \param scale Amount to scale this rotation
  public: void Scale(double scale);

  /// Multiplication operator
  /// \param qt Quatern for multiplication
  /// \return This quatern multiplied by the parameter
  public: Quatern operator*( const Quatern &qt ) const;

  /// See if a quatern is finite (e.g., not nan)
  /// \return True if quatern is finite
  public: bool IsFinite();

  /// Attributes of the quaternion 
  public: double u, x, y, z;

  /// Ostream operator
  /// \param out Ostream
  /// \param q Quatern to output
  public: friend std::ostream &operator<< (std::ostream &out, const Quatern &q);
};

#endif
/// @}
