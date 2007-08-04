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
/* Desc: Two dimensional vector
 * Author: Nate Koenig
 * Date: 3 Apr 2007
 * SVN: $Id:$
 */

#ifndef VECTOR2_HH
#define VECTOR2_HH

#include <iostream>
#include <fstream>

namespace gazebo
{
/// \addtogroup gazebo_server
/// \brief Generic x,y vector 
/// \{

/// \brief Generic x,y vector 
class Vector2
{
  /// \brief Constructor
  public: Vector2();

  /// \brief Constructor
  public: Vector2( const double &x, const double &y );

  /// \brief Constructor
  public: Vector2( const Vector2 &pt );

  /// \brief Destructor
  public: virtual ~Vector2();

  /// \brief Calc distance to the given point
  public: double Distance( const Vector2 &pt ) const;

  /// \brief  Normalize the vector length
  public: void Normalize();

  /// \brief Set the contents of the vector
  public: void Set(double x = 0, double y =0);

  /// \brief Return the cross product of this vector and pt
  public: Vector2 GetCrossProd(const Vector2 &pt) const;

  /// \brief Equal operator
  public: const Vector2 &operator=( const Vector2 &pt );

  /// \brief Equal operator
  public: const Vector2 &operator=( double value );

  /// \brief Addition operator
  public: Vector2 operator+( const Vector2 &pt ) const;

  /// \brief Addition operator
  public: const Vector2 &operator+=( const Vector2 &pt );

  /// \brief Subtraction operators 
  public: Vector2 operator-( const Vector2 &pt ) const;

  /// \brief Subtraction operators 
  public: const Vector2 &operator-=( const Vector2 &pt );

  /// \brief Division operators
  public: const Vector2 operator/( const Vector2 &pt ) const;

  /// \brief Division operators
  public: const Vector2 &operator/=( const Vector2 &pt );

  /// \brief Division operators
  public: const Vector2 operator/( double v ) const;

  /// \brief Division operators
  public: const Vector2 &operator/=( double v );

  /// \brief Multiplication operators
  public: const Vector2 operator*( const Vector2 &pt ) const;

  /// \brief Multiplication operators
  public: const Vector2 &operator*=( const Vector2 &pt );

  /// \brief Multiplication operators
  public: const Vector2 operator*( double v ) const;

  /// \brief Multiplication operators
  public: const Vector2 &operator*=( double v );

  /// \brief Equality operators
  public: bool operator==( const Vector2 &pt ) const;

  /// \brief Equality operators
  public: bool operator!=( const Vector2 &pt ) const;

  /// \brief See if a point is finite (e.g., not nan)
  public: bool IsFinite() const;

  /// \brief [] operator
  public: double operator[](unsigned int index) const;

  // \brief The location
  public: double x, y;

  /// \brief Ostream operator
  /// \param out Ostream
  /// \param pt Vector2 to output
  /// \return The Ostream
  public: friend std::ostream &operator<<( std::ostream &out, const gazebo::Vector2 &pt )
  {
    out << pt.x << " " << pt.y;

    return out;
  }

};

/// \}
}

#endif
