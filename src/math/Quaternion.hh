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

namespace gazebo
{
	namespace math
  {
  
  /// \addtogroup gazebo_server
  /// \brief A quaternion class
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
    public: Quaternion( const double &u, const double &x, const double &y, const double &z);
  
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
    public: Quaternion GetInverse() const;
  
    /// \brief Set the quatern to the identity
    public: void SetToIdentity();
  
    /// \brief Normalize the quaternion
    public: void Normalize();
  
    /// \brief Set the quaternion from an axis and angle
    /// \param x X axis 
    /// \param y Y axis
    /// \param z Z axis
    /// \param a Angle in radians
    public: void SetFromAxis(double x, double y, double z, double a);
  
    /// \brief Set this quaternion from another
    public: void Set(double u, double x, double y, double z);
           
    /// \brief Set the quaternion from Euler angles
    /// \param vec  Euler angle
    public: void SetFromEuler(const Vector3 &vec);
  
    /// \brief Return the rotation in Euler angles
    /// \return This quaternion as an Euler vector
    public: Vector3 GetAsEuler();
  
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
    public: Quaternion operator*( const Quaternion &qt ) const;
  
    /// \brief Multiplication operator
    /// \param qt Quaternion for multiplication
    /// \return This quatern multiplied by the parameter
    public: Quaternion operator*=( const Quaternion &qt );
  
    /// \brief Vector3 multiplication operator
    public: Vector3 operator*( const Vector3 &v ) const;
  
    /// \brief Rotate a vector using the quaternion
    /// \return The rotated vector
    public: Vector3 RotateVector(Vector3 vec) const;
  
    /// \brief Do the reverse rotation of a vector by this quaternion
    public: Vector3 RotateVectorReverse(Vector3 vec) const;
  
    /// \brief See if a quatern is finite (e.g., not nan)
    /// \return True if quatern is finite
    public: bool IsFinite() const;
  
    /// \brief Correct any nan
    public: void Correct();
  
    public: Vector3 GetXAxis() const;
    public: Vector3 GetYAxis() const;
    public: Vector3 GetZAxis() const;
  
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
      Vector3 v = const_cast<Quaternion*>(&q)->GetAsEuler();
      v.x = v.x * 180.0 / M_PI;
      v.y = v.y * 180.0 / M_PI;
      v.z = v.z * 180.0 / M_PI;
  
      if (std::isnan(v.x))
        v.x = 90.0;
      if (std::isnan(v.y))
        v.y = 90.0;
      if (std::isnan(v.z))
        v.z = 90.0;
  
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
