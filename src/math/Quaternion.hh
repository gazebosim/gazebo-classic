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
/* Desc: External interfaces for Gazebo
 * Author: Nate Koenig
 * Date: 03 Apr 2007
 * SVN: $Id$
 */

#ifndef QUATERN_HH
#define QUATERN_HH

#include <iostream>
#include <math.h> 
#include <cmath> 

#include "math/Angle.hh"
#include "math/Vector3.hh"
#include "math/Matrix3.hh"
#include "math/Matrix4.hh"

namespace gazebo
{
	namespace math
  {
  
  /// \addtogroup gazebo_math
  /// \{
  
  /// \brief A quaternion class
  class Quaternion
  {
    /// \brief Default Constructor
    public: Quaternion();
  
    /// \brief Constructor
    /// \param u U param
    /// \param x X param
    /// \param y Y param
    /// \param z Z param
    public: Quaternion( const double &w, const double &x, const double &y, const double &z);

    /// \brief Constructor
    public: Quaternion( const double &_roll, const double &_pitch, const double &_yaw );

    /// \brief Constructor
    public: Quaternion( const Vector3 &_rpy);
 
    /// \brief Copy constructor
    /// \param qt Quaternion to copy
    public: Quaternion( const Quaternion &qt );
  
    /// \brief Destructor
    public: ~Quaternion();
  
    /// \brief Equal operator
    /// \param qt Quaternion to copy
    public: const Quaternion &operator=(const Quaternion &qt);
  
    /// \brief Invert the quaternion
    public: void Invert();
  
    /// \brief Get the inverse of this quaternion
    /// \return Inverse quarenion
    public: inline Quaternion GetInverse() const
            {
              Quaternion q;
              double norm = this->w*this->w + this->x*this->x +
                            this->y*this->y + this->z*this->z;

              if (norm > 0.0)
              {
                q.w = this->w / norm;
                q.x = -this->x / norm;
                q.y = -this->y / norm;
                q.z = -this->z / norm;
              }
              return q;
            }
  
    /// \brief Set the quatern to the identity
    public: void SetToIdentity();

    public: Quaternion GetLog() const;
    public: Quaternion GetExp() const;
  
    /// \brief Normalize the quaternion
    public: void Normalize();
  
    /// \brief Set the quaternion from an axis and angle
    /// \param x X axis 
    /// \param y Y axis
    /// \param z Z axis
    /// \param a Angle in radians
    public: void SetFromAxis(double x, double y, double z, double a);

    /// \brief Set the quaternion from an axis and angle
    /// \param _axis Axis
    /// \param _a Angle in radians
    public: void SetFromAxis(const Vector3 &_axis, double _a);
  
    /// \brief Set this quaternion from another
    public: void Set(double u, double x, double y, double z);
           
    /// \brief Set the quaternion from Euler angles
    /// \param vec  Euler angle
    public: void SetFromEuler(const Vector3 &vec);
  
    /// \brief Return the rotation in Euler angles
    /// \return This quaternion as an Euler vector
    public: Vector3 GetAsEuler() const;
  
    /// \brief Convert euler angles to quatern.
    public: static Quaternion EulerToQuaternion( const Vector3 &vec );
  
    /// \brief Convert euler angles to quatern.
    public: static Quaternion EulerToQuaternion( double x, double y, double z);
  
    /// \brief Get the Euler roll angle in radians
    public: double GetRoll();
  
    /// \brief Get the Euler pitch angle in radians
    public: double GetPitch();
  
    /// \brief Get the Euler yaw angle in radians
    public: double GetYaw();
  
    /// \brief Return rotation as axis and angle
    public: void GetAsAxis(Vector3 &axis, double &angle) const;
  
    /// \brief Scale a Quaternionion
    /// \param scale Amount to scale this rotation
    public: void Scale(double scale);
  
    /// \brief Addition operator
    /// \param qt Quaternion for addition
    /// \return This quatern + qt
    public: Quaternion operator+( const Quaternion &qt ) const;
  
    /// \brief Addition operator
    /// \param qt Quaternion for addition
    /// \return This quatern + qt
    public: Quaternion operator+=( const Quaternion &qt );
  
    /// \brief Substraction operator
    /// \param qt Quaternion for substraction
    /// \return This quatern - qt
    public: Quaternion operator-( const Quaternion &qt ) const;
  
    /// \brief Substraction operator
    /// \param qt Quaternion for substraction
    /// \return This quatern - qt
    public: Quaternion operator-=( const Quaternion &qt );
  
    /// \brief Multiplication operator
    /// \param qt Quaternion for multiplication
    /// \return This quatern multiplied by the parameter
    public: inline Quaternion operator*( const Quaternion &_q ) const
            {
              return Quaternion(
                  this->w*_q.w - this->x*_q.x - this->y*_q.y - this->z*_q.z,
                  this->w*_q.x + this->x*_q.w + this->y*_q.z - this->z*_q.y,
                  this->w*_q.y - this->x*_q.z + this->y*_q.w + this->z*_q.x,
                  this->w*_q.z + this->x*_q.y - this->y*_q.x + this->z*_q.w);
            }

    /// \brief Multipcation operator
    /// \param _f Double factor
    /// \return Quaternion multiplied by _f
    public: Quaternion operator*( const double &_f ) const;
  
    /// \brief Multiplication operator
    /// \param qt Quaternion for multiplication
    /// \return This quatern multiplied by the parameter
    public: Quaternion operator*=( const Quaternion &qt );
  
    /// \brief Vector3 multiplication operator
    public: Vector3 operator*( const Vector3 &v ) const;
  
    /// \brief Equality operator
    /// \param _qt Quaternion for comparison
    /// \return True if equal
    public: bool operator==(const Quaternion &_qt) const;

    /// \brief Inequality operator
    /// \param _qt Quaternion for comparison
    /// \return True if not equal
    public: bool operator!=(const Quaternion &_qt) const;

    /// \brief Negate operator
    public: Quaternion operator-() const;

    /// \brief Rotate a vector using the quaternion
    /// \return The rotated vector
    public: inline Vector3 RotateVector(const Vector3 &_vec) const
            {
              Quaternion tmp(0.0, _vec.x, _vec.y, _vec.z);
              tmp = (*this) * (tmp * this->GetInverse());
              return Vector3(tmp.x, tmp.y, tmp.z);
            }
  
    /// \brief Do the reverse rotation of a vector by this quaternion
    public: Vector3 RotateVectorReverse(Vector3 vec) const;
  
    /// \brief See if a quatern is finite (e.g., not nan)
    /// \return True if quatern is finite
    public: bool IsFinite() const;
  
    /// \brief Correct any nan
    public: inline void Correct()
            {
              if (!finite(this->x))
                this->x = 0;
              if (!finite(this->y))
                this->y = 0;
              if (!finite(this->z))
                this->z = 0;
              if (!finite(this->w))
                this->w = 1;

              if (this->w == 0 && this->x == 0 && this->y == 0 && this->z == 0)
                this->w = 1;
            }

    /// \brief Get the quaternion as a 3x3 matrix
    public: Matrix3 GetAsMatrix3() const;

     /// \brief Get the quaternion as a 4x4 matrix
    public: Matrix4 GetAsMatrix4() const;
  
    public: Vector3 GetXAxis() const;
    public: Vector3 GetYAxis() const;
    public: Vector3 GetZAxis() const;
  
    /// \brief Round all values to _precision decimal places
    public: void Round(int _precision);

    /// \brief Dot product
    public: double Dot(const Quaternion &_q) const;

    /// \brief Spherical quadratic interpolation
    public: static Quaternion Squad(double _fT, const Quaternion &_rkP,
                const Quaternion &_rkA, const Quaternion &_rkB,
                Quaternion &_rkQ, bool _shortestPath = false);

    /// \brief Spherical linear interpolation
    public: static Quaternion Slerp (double _fT, const Quaternion &_rkP,
                const Quaternion &_rkQ, bool _shortestPath = false);
                  

    /// \brief Attributes of the quaternion 
    public: double w;
  
    /// \brief Attributes of the quaternion 
    public: double x;
  
    /// \brief Attributes of the quaternion 
    public: double y;
  
    /// \brief Attributes of the quaternion 
    public: double z;
  
    /// \brief Ostream operator
    /// \param out Ostream
    /// \param q Quaternion to output
    /// \return The ostream
    public: friend  std::ostream &operator<<( std::ostream &out, 
                const gazebo::math::Quaternion &q )
    {
      Vector3 v( q.GetAsEuler() );
      //v.x = v.x * 180.0 / M_PI;
      //v.y = v.y * 180.0 / M_PI;
      //v.z = v.z * 180.0 / M_PI;
  
      /*if (std::isnan(v.x))
        v.x = 90.0;
      if (std::isnan(v.y))
        v.y = 90.0;
      if (std::isnan(v.z))
        v.z = 90.0;
        */
  
      out << v.x << " " << v.y << " " << v.z;
  
      return out;
    }
  
    /// \brief Istream operator
    /// \param in Ostream
    /// \param q Quaternion to read values into
    /// \return The istream
    public: friend std::istream &operator>>( std::istream &in, 
                                             gazebo::math::Quaternion &q )
    {
      Angle r, p, y;
  
      // Skip white spaces
      in.setf( std::ios_base::skipws );
      in >> r >> p >> y;

      q.SetFromEuler(Vector3(*r,*p,*y));
  
      return in;
    }
  
  
  };
  
  /// \}
  }

}
#endif
