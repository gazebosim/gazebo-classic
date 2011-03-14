/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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
  namespace common
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
}

#endif
