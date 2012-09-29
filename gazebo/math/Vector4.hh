/*
 * Copyright 2011 Nate Koenig
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
/* Desc: 4 tuple
 * Author: Nate Koenig
 * Date: 19 Aug 2008
 */

#ifndef VECTOR4_HH
#define VECTOR4_HH

#include <iostream>
#include <fstream>
#include "math/Matrix4.hh"

namespace gazebo
{
  namespace math
  {
    /// \addtogroup gazebo_math
    /// \{

    /// \brief double Generic x, y, z, w vector
    class Vector4
    {
      /// \brief Constructor
      public: Vector4();

      /// \brief Constructor
      public: Vector4(const double &_x, const double &_y, const double &_z,
                      const double &_w);

      /// \brief Constructor
      public: Vector4(const Vector4 &_pt);

      /// \brief Destructor
      public: virtual ~Vector4();

      /// \brief Calc distance to the given point
      public: double Distance(const Vector4 &_pt) const;

      /// \brief Returns the length (magnitude) of the vector
      public: double GetLength() const;

      /// \brief Return the square of the length (magnitude) of the vector
      public: double GetSquaredLength() const;

      /// \brief Normalize the vector length
      public: void Normalize();

      /// \brief Set the contents of the vector
      public: void Set(double _x = 0, double _y = 0 , double _z = 0,
                       double _w = 0);

      /// \brief Equal operator
      public: Vector4 &operator =(const Vector4 &pt);

      /// \brief Equal operator
      public: Vector4 &operator =(double value);

      /// \brief Addition operator
      public: Vector4 operator+(const Vector4 &pt) const;

      /// \brief Addition operator
      public: const Vector4 &operator+=(const Vector4 &pt);

      /// \brief Subtraction operators
      public: Vector4 operator-(const Vector4 &pt) const;

      /// \brief Subtraction operators
      public: const Vector4 &operator-=(const Vector4 &pt);

      /// \brief Division operators
      public: const Vector4 operator/(const Vector4 &pt) const;

      /// \brief Division operators
      public: const Vector4 &operator/=(const Vector4 &pt);

      /// \brief Division operators
      public: const Vector4 operator/(double v) const;

      /// \brief Division operators
      public: const Vector4 &operator/=(double v);

      /// \brief Multiplication operators
      public: const Vector4 operator*(const Vector4 &pt) const;

      public: const Vector4 operator*(const Matrix4 &_m) const;

      /// \brief Multiplication operators
      public: const Vector4 &operator*=(const Vector4 &pt);

      /// \brief Multiplication operators
      public: const Vector4 operator*(double v) const;

      /// \brief Multiplication operators
      public: const Vector4 &operator*=(double v);

      /// \brief Equality operators
      public: bool operator ==(const Vector4 &pt) const;

      /// \brief Equality operators
      public: bool operator!=(const Vector4 &pt) const;

      /// \brief See if a point is finite (e.g., not nan)
      public: bool IsFinite() const;

      /// \brief [] operator
      public: double operator[](unsigned int index) const;

      /// X value
      public: double x;

      /// Y value
      public: double y;

      /// Z value
      public: double z;

      /// W value
      public: double w;

      /// \brief Ostream operator
      /// \param out Ostream
      /// \param pt Vector4 to output
      /// \return The Ostream
      public: friend std::ostream &operator<<(std::ostream &_out,
                  const gazebo::math::Vector4 &_pt)
      {
        _out << _pt.x << " " << _pt.y << " " << _pt.z << " " << _pt.w;
        return _out;
      }

      /// \brief Istream operator
      /// \param in Ostream
      /// \param pt Vector4 to read values into
      /// \return The istream
      public: friend std::istream &operator>>(std::istream &_in,
                  gazebo::math::Vector4 &_pt)
      {
        // Skip white spaces
        _in.setf(std::ios_base::skipws);
        _in >> _pt.x >> _pt.y >> _pt.z >> _pt.w;
        return _in;
      }
    };
    /// \}
  }
}
#endif



