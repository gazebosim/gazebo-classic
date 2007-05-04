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
/* Desc: The world; all models are collected here
 * Author: Nate Koenig
 * Date: 3 Apr 2007
 * SVN: $Id$
 */

#ifndef VECTOR3_HH
#define VECTOR3_HH

#include <iostream>
#include <fstream>

namespace gazebo
{
/// @addtogroup gazebocore
/// @{

/// Generic x,y,z vector 
class Vector3
{
  /// Constructors
  public: Vector3();
  public: Vector3( const double &x, const double &y, const double &z );
  public: Vector3( const Vector3 &pt );

  /// Destructor
  public: virtual ~Vector3();

  /// Calc distance to the given point
  public: double Distance( const Vector3 &pt ) const;

  /// Normalize the vector length
  public: void Normalize();

  /// Set the contents of the vector
  public: void Set(double x = 0, double y =0 , double z = 0);

  /// Return the cross product of this vector and pt
  public: Vector3 GetCrossProd(const Vector3 &pt) const;

  /// Equal operator
  public: const Vector3 &operator=( const Vector3 &pt );

  /// Addition operators
  public: Vector3 operator+( const Vector3 &pt ) const;
  public: const Vector3 &operator+=( const Vector3 &pt );

  /// Subtraction operators 
  public: Vector3 operator-( const Vector3 &pt ) const;
  public: const Vector3 &operator-=( const Vector3 &pt );

  /// Division operators
  public: const Vector3 operator/( const Vector3 &pt ) const;
  public: const Vector3 &operator/=( const Vector3 &pt );

  /// Multiplication operators
  public: const Vector3 operator*( const Vector3 &pt ) const;
  public: const Vector3 &operator*=( const Vector3 &pt );

  /// Equality operators
  public: bool operator==( const Vector3 &pt ) const;
  public: bool operator!=( const Vector3 &pt ) const;

  /// See if a point is finite (e.g., not nan)
  public: bool IsFinite() const;

  // The location
  public: double x, y, z;

  /// Ostream operator
  /// \param out Ostream
  /// \param pt Vector3 to output
  /// \return The Ostream
  public: friend std::ostream &operator<<( std::ostream &out, const gazebo::Vector3 &pt )
  {
    out << pt.x << " " << pt.y << " " << pt.z;

    return out;
  }

};

/// @}
}

#endif
