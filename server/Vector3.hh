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
/// \addtogroup gazebo_server
/// \brief Generic x,y,z vector 
/// \{

/// \brief Generic x,y,z vector 
class Vector3
{
  /// \brief Constructor
  public: Vector3();

  /// \brief Constructor
  public: Vector3( const double &x, const double &y, const double &z );

  /// \brief Constructor
  public: Vector3( const Vector3 &pt );

  /// \brief Destructor
  public: virtual ~Vector3();

  /// \brief Calc distance to the given point
  public: double Distance( const Vector3 &pt ) const;

  /// \brief Returns the length (magnitude) of the vector
  public: double GetLength() const;

  /// \brief Return the square of the length (magnitude) of the vector
  public: double GetSquaredLength() const;

  /// \brief Normalize the vector length
  public: void Normalize();

  /// \brief Set the contents of the vector
  public: void Set(double x = 0, double y =0 , double z = 0);

  /// \brief Return the cross product of this vector and pt
  public: Vector3 GetCrossProd(const Vector3 &pt) const;

  /// \brief Return a vector that is perpendicular to this one.
  public: Vector3 GetPerpendicular() const;

  /// \brief Equal operator
  public: const Vector3 &operator=( const Vector3 &pt );

  /// \brief Equal operator
  public: const Vector3 &operator=( double value );

  /// \brief Addition operator
  public: Vector3 operator+( const Vector3 &pt ) const;

  /// \brief Addition operator
  public: const Vector3 &operator+=( const Vector3 &pt );

  /// \brief Subtraction operators 
  public: Vector3 operator-( const Vector3 &pt ) const;

  /// \brief Subtraction operators 
  public: const Vector3 &operator-=( const Vector3 &pt );

  /// \brief Division operators
  public: const Vector3 operator/( const Vector3 &pt ) const;

  /// \brief Division operators
  public: const Vector3 &operator/=( const Vector3 &pt );

  /// \brief Division operators
  public: const Vector3 operator/( double v ) const;

  /// \brief Division operators
  public: const Vector3 &operator/=( double v );

  /// \brief Multiplication operators
  public: const Vector3 operator*( const Vector3 &pt ) const;

  /// \brief Multiplication operators
  public: const Vector3 &operator*=( const Vector3 &pt );

  /// \brief Multiplication operators
  public: const Vector3 operator*( double v ) const;

  /// \brief Multiplication operators
  public: const Vector3 &operator*=( double v );

  /// \brief Equality operators
  public: bool operator==( const Vector3 &pt ) const;

  /// \brief Equality operators
  public: bool operator!=( const Vector3 &pt ) const;

  /// \brief See if a point is finite (e.g., not nan)
  public: bool IsFinite() const;

  /// \brief [] operator
  public: double operator[](unsigned int index) const;

  /// \brief X location
  public: double x;

  /// \brief Y location
  public: double y;

  /// \brief Z location
  public: double z;

  /// \brief Ostream operator
  /// \param out Ostream
  /// \param pt Vector3 to output
  /// \return The Ostream
  public: friend std::ostream &operator<<( std::ostream &out, const gazebo::Vector3 &pt )
  {
    out << pt.x << " " << pt.y << " " << pt.z;

    return out;
  }

  /// \brief Istream operator
  /// \param in Ostream
  /// \param pt Vector3 to read values into
  /// \return The istream
  public: friend std::istream &operator>>( std::istream &in, gazebo::Vector3 &pt )
  {
    // Skip white spaces
    in.setf( std::ios_base::skipws );
    in >> pt.x >> pt.y >> pt.z;
    return in;
  }


};

/// \}
}

#endif
