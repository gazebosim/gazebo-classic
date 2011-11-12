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
/* Desc: The world; all models are collected here
 * Author: Nate Koenig
 * Date: 3 Apr 2007
 */

#ifndef VECTOR3_HH
#define VECTOR3_HH

#include <math.h>
#include <iostream>
#include <fstream>

namespace gazebo
{
	namespace math
  {
    /// \addtogroup gazebo_math
    /// \{
    
    /// \brief Generic double x,y,z vector 
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
    
      /// \brief Round to near whole number, return the result.
      public: Vector3 Round();
    
      /// \brief Get a rounded version of this vector
      public: Vector3 GetRounded() const;
    
      /// \brief Set the contents of the vector
      public: inline void Set(double x = 0, double y =0 , double z = 0)
              {
                this->x = x;
                this->y = y;
                this->z = z;
              }
    
      /// \brief Return the cross product of this vector and pt
      public: Vector3 GetCrossProd(const Vector3 &pt) const;
    
      /// \brief Return the dot product of this vector and pt
      public: double GetDotProd(const Vector3 &pt) const;
    
      /// \breif Get the absolute value of the vector
      public: Vector3 GetAbs() const;
    
      /// \brief Return a vector that is perpendicular to this one.
      public: Vector3 GetPerpendicular() const;
    
      /// \brief Get a normal vector to a triangle
      public: static Vector3 GetNormal(const Vector3 &v1, const Vector3 &v2, 
                                       const Vector3 &v3);
    
      /// \brief Get distance to a plane, given a direction. Treats this 
      ///        vector as a ray
      public: double GetDistToPlane(const Vector3 &_dir, 
                                    const Vector3 &_planeNormal, 
                                    double _d) const;
      /// \brief Get distance to a line
      public: double GetDistToLine(const Vector3 &_pt1, const Vector3 &_pt2);
    
      /// \brief Set this vector's components to the maximum of itself and the 
      ///        passed in vector
      public: void SetToMax(const Vector3 & v);
    
      /// \brief Set this vector's components to the minimum of itself and the 
      ///        passed in vector
      public: void SetToMin(const Vector3 & v);
    
      /// \brief Equal operator
      public: const Vector3 &operator=( const Vector3 &pt );
    
      /// \brief Equal operator
      public: const Vector3 &operator=( double value );
    
      /// \brief Addition operator
      public: Vector3 operator+( const Vector3 &pt ) const;
    
      /// \brief Addition operator
      public: const Vector3 &operator+=( const Vector3 &pt );
    
      /// \brief Subtraction operators 
      public: inline Vector3 operator-( const Vector3 &pt ) const
              {
                return Vector3(this->x - pt.x, this->y - pt.y, this->z - pt.z);
              }
    
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
    
      /// \brief Corrects any nan values
      public: inline void Correct()
              {
                if (!finite(this->x))
                  this->x = 0;
                if (!finite(this->y))
                  this->y = 0;
                if (!finite(this->z))
                  this->z = 0;
              }
    
      /// \brief [] operator
      public: double operator[](unsigned int index) const;

      /// \brief Round all values to _precision decimal places
      public: void Round(int _precision);

      /// \brief Returns true if the two vectors are exacatly equal 
      public: bool Equal( const Vector3 &_v ) const;

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
      public: friend std::ostream &operator<<( std::ostream &out, const gazebo::math::Vector3 &pt )
      {
        out << pt.x << " " << pt.y << " " << pt.z;
    
        return out;
      }
    
      /// \brief Istream operator
      /// \param in Ostream
      /// \param pt Vector3 to read values into
      /// \return The istream
      public: friend std::istream &operator>>( std::istream &in, gazebo::math::Vector3 &pt )
      {
        // Skip white spaces
        in.setf( std::ios_base::skipws );
        in >> pt.x >> pt.y >> pt.z;
        return in;
      }
    };
    
    /// \}
  }

}
#endif
