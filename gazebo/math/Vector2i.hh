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
 * WITHOUint WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/* Desc: Two dimensional vector
 * Author: Nate Koenig
 * Date: 3 Apr 2007
 */

#ifndef VECTOR2I_HH
#define VECTOR2I_HH

#include <math.h>
#include <iostream>
#include <fstream>

namespace gazebo
{
  namespace math
  {
    /// \addtogroup gazebo_math
    /// \{

    /// \brief Generic integer x, y vector
    class Vector2i
    {
      /// \brief Constructor
      public: Vector2i();

      /// \brief Constructor
      public: Vector2i(const int &_x, const int &_y);

      /// \brief Constructor
      public: Vector2i(const Vector2i &_pt);

      /// \brief Destructor
      public: virtual ~Vector2i();

      /// \brief Calc distance to the given point
      public: int Distance(const Vector2i &_pt) const;

      /// \brief  Normalize the vector length
      public: void Normalize();

      /// \brief Set the contents of the vector
      public: void Set(int _x, int _y);

      /// \brief Return the cross product of this vector and pt
      public: Vector2i Cross(const Vector2i &_pt) const;

      /// \brief Equal operator
      public: Vector2i &operator =(const Vector2i &pt);

      /// \brief Equal operator
      public: const Vector2i &operator =(int value);

      /// \brief Addition operator
      public: Vector2i operator+(const Vector2i &pt) const;

      /// \brief Addition operator
      public: const Vector2i &operator+=(const Vector2i &pt);

      /// \brief Subtraction operators
      public: Vector2i operator-(const Vector2i &pt) const;

      /// \brief Subtraction operators
      public: const Vector2i &operator-=(const Vector2i &pt);

      /// \brief Division operators
      public: const Vector2i operator/(const Vector2i &pt) const;

      /// \brief Division operators
      public: const Vector2i &operator/=(const Vector2i &pt);

      /// \brief Division operators
      public: const Vector2i operator/(int v) const;

      /// \brief Division operators
      public: const Vector2i &operator/=(int v);

      /// \brief Multiplication operators
      public: const Vector2i operator*(const Vector2i &pt) const;

      /// \brief Multiplication operators
      public: const Vector2i &operator*=(const Vector2i &pt);

      /// \brief Multiplication operators
      public: const Vector2i operator*(int v) const;

      /// \brief Multiplication operators
      public: const Vector2i &operator*=(int v);

      /// \brief Equality operators
      public: bool operator ==(const Vector2i &pt) const;

      /// \brief Equality operators
      public: bool operator!=(const Vector2i &pt) const;

      /// \brief See if a point is finite (e.g., not nan)
      public: bool IsFinite() const;

      /// \brief [] operator
      public: int operator[](unsigned int index) const;

      /// \brief x data
      public: int x;

      /// \brief y data
      public: int y;

      /// \brief Ostream operator
      /// \param out Ostream
      /// \param pt Vector2i to output
      /// \return The Ostream
      public: friend std::ostream &operator<<(std::ostream &_out,
                  const gazebo::math::Vector2i &_pt)
      {
        _out << _pt.x << " " << _pt.y;
        return _out;
      }

      /// \brief Istream operator
      /// \param in Ostream
      /// \param pt Vector3 to read values into
      /// \return The istream
      public: friend std::istream &operator>>(std::istream &_in,
                  gazebo::math::Vector2i &_pt)
      {
        // Skip white spaces
        _in.setf(std::ios_base::skipws);
        _in >> _pt.x >> _pt.y;
        return _in;
      }
    };

  /// \}
  }
}
#endif



