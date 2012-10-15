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
#ifndef BOX_HH
#define BOX_HH

#include <iostream>
#include "math/Vector3.hh"

namespace gazebo
{
  namespace math
  {
    /// \addtogroup gazebo_math
    /// \{

    /// \brief Mathematical representation of a box and related functions.
    class Box
    {
      /// \brief Default constructor
      public: Box();

      /// \brief Constructor
      /// \param _min Minimum corner of the box
      /// \param _max Maximum corner of the box
      public: Box(const Vector3 &_min, const Vector3 &_max);

      /// \brief Copy Constructor
      /// \param _b Box to copy
      public: Box(const Box &_b);

      /// \brief Destructor
      public: virtual ~Box();

      /// \brief Get the length along the x dimension
      /// \return Double value of the length in the x dimension
      public: double GetXLength() const;

      /// \brief Get the length along the y dimension
      /// \return Double value of the length in the y dimension
      public: double GetYLength() const;

      /// \brief Get the length along the z dimension
      /// \return Double value of the length in the z dimension
      public: double GetZLength() const;

      /// \brief Get the size of the box
      /// \return Size of the box
      public: math::Vector3 GetSize() const;

      /// \brief Get the box center
      /// \return The center position of the box
      public: math::Vector3 GetCenter() const;

      /// \brief Merge a box with this box
      /// \param _box Box to add to this box
      public: void Merge(const Box &_box);

      /// \brief Equal operator. Set this box to the parameter
      /// \param _b Box to copy
      /// \return The new box.
      public: Box &operator =(const Box &_b);

      /// \brief Addition operator. result = this + _b
      /// \param _b Box to add
      /// \return The new box
      public: Box operator+(const Box &_b) const;

      /// \brief Addition set operator. this = this + _b
      /// \param _b Box to add
      /// \return This new box
      public: const Box &operator+=(const Box &_b);

      /// \brief Equality test operatoer
      /// \param _b Box to test
      /// \return True if equal
      public: bool operator==(const Box &_b);

      /// \brief Subtract a vector from the min and max values
      /// \param _v The vector to use during subtraction
      /// \return The new box
      public: Box operator-(const Vector3 &_v);

      /// \brief Output operator
      /// \param _out Output stream
      /// \param _b Box to output to the stream
      /// \return The stream
      public: friend std::ostream &operator<<(std::ostream &_out,
                                               const gazebo::math::Box &_b)
      {
        _out << "Min[" << _b.min << "] Max[" << _b.max << "]";

        return _out;
      }

      /// \brief Minimum corner of the box
      public: Vector3 min;

      /// \brief Maximum corner of the box
      public: Vector3 max;

      private: enum Extent {EXTENT_NULL, EXTENT_FINITE};
      private: Extent extent;
    };
    /// \}
  }
}

#endif



