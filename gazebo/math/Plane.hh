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
#ifndef PLANE_HH
#define PLANE_HH

#include "math/Vector3.hh"
#include "math/Vector2d.hh"

namespace gazebo
{
  namespace math
  {
    /// \addtogroup gazebo_math
    /// \{

    /// \brief A plane and related functions.
    class Plane
    {
      /// \brief Constructor
      public: Plane();

      /// \brief Constructor
      /// \param _normal The plane normal
      /// \param _offset Offset along the normal
      public: Plane(const Vector3 &_normal, double _offset = 0.0);

      /// \brief Constructor
      /// \param _normal The plane normal
      /// \param _size Size of the plane
      /// \param _offset Offset along the normal
      public: Plane(const Vector3 &_normal, const Vector2d &_size,
                    double _offset);

      /// \brief Destructor
      public: virtual ~Plane();

      /// \brief Set the plane
      /// \param _normal The plane normal
      /// \param _size Size of the plane
      /// \param _offset Offset along the normal
      public: void Set(const Vector3 &_normal, const Vector2d &_size,
                       double offset);

      /// \brief Get distance to the plane give an origin and direction
      public: double Distance(const Vector3 &_origin,
                              const Vector3 &_dir) const;

      /// \brief Equal operator
      /// \param _p Plane values
      public: Plane &operator =(const Plane &_p);

      /// \brief Plane normal
      public: Vector3 normal;

      /// \brief Plane size
      public: Vector2d size;

      /// \brief Plane offset
      public: double d;
    };
    /// \}
  }
}
#endif



