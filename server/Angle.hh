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
/* Desc: Angle class
 * Author: Nate Koenig
 * Date: 18 Aug 2008
 * SVN: $Id$
 */

#ifndef ANGLE_HH
#define ANGLE_HH

#include <iostream>
#include <math.h>

namespace gazebo
{

/// \addtogroup gazebo_server
/// \brief Angle class
/// \{

/// \brief Angle class
  class Angle
  {
    /// \brief Constructor
    public: Angle();

    /// \brief Constructor
    public: Angle(double radian);

    /// \brief Copy constructor
    public: Angle(const Angle &angle);
  
    /// \brief Destructor
    public: virtual ~Angle();
  
    /// \brief Set the value from an angle in radians
    public: void SetFromRadian( double radian );
  
    /// \brief Set the value from an angle in degrees
    public: void SetFromDegree( double degree );
  
    /// \brief Get the angle in radians
    public: double GetAsRadian() const;
  
    /// \brief Get the angle in degrees
    public: double GetAsDegree() const;

    /// \brief Normalize the angle
    public: void Normalize();

    /// \brief Dereference operator
    public: inline double operator*() { return value; }

    /// \brief Substraction
    public: Angle operator-(const Angle &angle) const;
    /// \brief Addition
    public: Angle operator+(const Angle &angle) const;
    /// \brief Multiplication
    public: Angle operator*(const Angle &angle) const;
    /// \brief Division
    public: Angle operator/(const Angle &angle) const;

    /// \brief Add set
    public: Angle operator-=(const Angle &angle);
    /// \brief Sub set
    public: Angle operator+=(const Angle &angle);
    /// \brief Mul set
    public: Angle operator*=(const Angle &angle);
    /// \brief Div set
    public: Angle operator/=(const Angle &angle);

    /// \brief Equality
    public: bool operator==(const Angle &angle) const;

    /// \brief Inequality
    public: bool operator!=(const Angle &angle) const;

    /// \brief Less
    public: bool operator<(const Angle &angle) const;
    /// \brief Less equal
    public: bool operator<=(const Angle &angle) const;

    /// \brief Greater
    public: bool operator>(const Angle &angle) const;
    /// \brief Greater equal
    public: bool operator>=(const Angle &angle) const;

    /// \brief Ostream operator. Outputs in degrees
    /// \param out Ostream
    /// \param pt Angle to output
    /// \return The Ostream
    public: friend std::ostream &operator<<( std::ostream &out, const gazebo::Angle &a )
    {
      out << a.GetAsDegree();
      return out;
    }
  
    /// \brief Istream operator. Assumes input is in degrees
    /// \param in Ostream
    /// \param pt Angle to read value into
    /// \return The istream
    public: friend std::istream &operator>>( std::istream &in, gazebo::Angle &a )
    {
      // Skip white spaces
      in.setf( std::ios_base::skipws );
      in >> a.value;
      a.value = a.value * M_PI / 180.0; 
      return in;
    }
  
    /// The angle in radians
    private: double value;
  };
  
/// \}
}

#endif
