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
#ifndef BOX_HH
#define BOX_HH

#include <iostream>
#include "common/Vector3.hh"

namespace gazebo
{
  namespace common
  {
    class Box
    {
      /// \brief Default constructor
      public: Box();

      /// \brief Constructor
      public: Box (const Vector3 &min, const Vector3 &max);

      /// \brief Copy Constructor
      public: Box( const Box &b );

      /// \brief Destructor
      public: virtual ~Box();

      /// \brief Get the length along the x dimension
      public: double GetXLength();

      /// \brief Get the length along the y dimension
      public: double GetYLength();

      /// \brief Get the length along the z dimension
      public: double GetZLength();

      /// \brief Merge a box with this box
      public: void Merge(const Box &box);

      /// \brief Equal operator
      public: const Box &operator=( const Box &b );

      public: Box operator+( const Box &b ) const;

      public: const Box &operator+=( const Box &b );

      public: friend std::ostream &operator<<( std::ostream &out, const gazebo::common::Box &b )
      {
        out << "Min[" << b.min << "] Max[" << b.max << "]";

        return out;
      }
 
      public: Vector3 min, max;
    };
  }
}

#endif
