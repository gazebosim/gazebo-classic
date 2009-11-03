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

#ifndef QUATERN_HH
#define QUATERN_HH

#include <iostream>
#include <math.h> 
#include <cmath> 

#include "Angle.hh"
#include "Vector3.hh"

namespace gazebo
{

/// \addtogroup gazebo_server
/// \brief A quaternion class
/// \{

/// \brief A quaternion class
class Quatern
{
  /// \brief Default Constructor
  public: Quatern();

  /// \brief Constructor
  /// \param u U param
  /// \param x X param
  /// \param y Y param
  /// \param z Z param
  public: Quatern( const double &u, const double &x, const double &y, const double &z);

  /// \brief Copy constructor
  /// \param qt Quatern to copy
  public: Quatern( const Quatern &qt );

  /// \brief Destructor
  public: ~Quatern();

  /// \brief Equal operator
  /// \param qt Quatern to copy
  public: const Quatern &operator=(const Quatern &qt);

  /// \brief Invert the quaternion
  public: void Invert();

  /// \brief Get the inverse of this quaternion
  /// \return Inverse quarenion
  public: Quatern GetInverse() const;

  /// \brief Set the quatern to the identity
  public: void SetToIdentity();

  /// \brief Normalize the quaternion
  public: void Normalize();

  /// \brief Set the quaternion from an axis and angle
  /// \param x X axis 
  /// \param y Y axis
  /// \param z Z axis
  /// \param a Angle in radians
  public: void SetFromAxis(double x, double y, double z, double a);

  /// \brief Set this quaternion from another
  public: void Set(double u, double x, double y, double z);
         
  /// \brief Set the quaternion from Euler angles
  /// \param vec  Euler angle
  public: void SetFromEuler(const Vector3 &vec);

  /// \brief Return the rotation in Euler angles
  /// \return This quaternion as an Euler vector
  public: Vector3 GetAsEuler();

  /// \brief Get the Euler roll angle in radians
  public: double GetRoll();

  /// \brief Get the Euler pitch angle in radians
  public: double GetPitch();

  /// \brief Get the Euler yaw angle in radians
  public: double GetYaw();

  /// \brief Return rotation as axis and angle (x, y, y, rotation)
  /// \return This quaternion as an axis-angle
  public: Quatern GetAsAxis();

  /// \brief Scale a Quaternion
  /// \param scale Amount to scale this rotation
  public: void Scale(double scale);

  /// \brief Addition operator
  /// \param qt Quatern for addition
  /// \return This quatern + qt
  public: Quatern operator+( const Quatern &qt ) const;

  /// \brief Addition operator
  /// \param qt Quatern for addition
  /// \return This quatern + qt
  public: Quatern operator+=( const Quatern &qt );

  /// \brief Substraction operator
  /// \param qt Quatern for substraction
  /// \return This quatern - qt
  public: Quatern operator-( const Quatern &qt ) const;

  /// \brief Substraction operator
  /// \param qt Quatern for substraction
  /// \return This quatern - qt
  public: Quatern operator-=( const Quatern &qt );

  /// \brief Multiplication operator
  /// \param qt Quatern for multiplication
  /// \return This quatern multiplied by the parameter
  public: Quatern operator*( const Quatern &qt ) const;

  /// \brief Multiplication operator
  /// \param qt Quatern for multiplication
  /// \return This quatern multiplied by the parameter
  public: Quatern operator*=( const Quatern &qt );

  /// \brief Vector3 multiplication operator
  public: Vector3 operator*( const Vector3 &v ) const;

  /// \brief Rotate a vector using the quaternion
  /// \return The rotated vector
  public: Vector3 RotateVector(Vector3 vec) const;

  /// \brief See if a quatern is finite (e.g., not nan)
  /// \return True if quatern is finite
  public: bool IsFinite() const;

  /// \brief Attributes of the quaternion 
  public: double u;

  /// \brief Attributes of the quaternion 
  public: double x;

  /// \brief Attributes of the quaternion 
  public: double y;

  /// \brief Attributes of the quaternion 
  public: double z;

  /// \brief Ostream operator
  /// \param out Ostream
  /// \param q Quatern to output
  /// \return The ostream
  public: friend  std::ostream &operator<<( std::ostream &out, const gazebo::Quatern &q )
  {
    Vector3 v = const_cast<Quatern*>(&q)->GetAsEuler();
    v.x = v.x * 180.0 / M_PI;
    v.y = v.y * 180.0 / M_PI;
    v.z = v.z * 180.0 / M_PI;

    if (std::isnan(v.x))
      v.x = 90.0;
    if (std::isnan(v.y))
      v.y = 90.0;
    if (std::isnan(v.z))
      v.z = 90.0;

    out << v.x << " " << v.y << " " << v.z;

    return out;
  }

  /// \brief Istream operator
  /// \param in Ostream
  /// \param q Quatern to read values into
  /// \return The istream
  public: friend std::istream &operator>>( std::istream &in, gazebo::Quatern &q )
  {
    Angle r, p, y;

    // Skip white spaces
    in.setf( std::ios_base::skipws );
    in >> r >> p >> y;

    q.SetFromEuler(Vector3(*r,*p,*y));

    return in;
  }


};

/// \}
}

#endif
