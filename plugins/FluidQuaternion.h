/*
 * Copyright 2012 Open Source Robotics Foundation
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

#ifndef QUATERNION_H_
#define QUATERNION_H_

#include <math.h>
#include "vector_functions.h"

namespace fluidix
{
	class Quaternion
	{
	    /// \brief Default Constructor
	    public: Quaternion()
	    : w(1), x(0), y(0), z(0)
	    {

	    }

	    /// \brief Constructor
	    public: Quaternion(const float &_w, const float &_x, const float &_y,
	    		const float &_z)
	    : w(_w), x(_x), y(_y), z(_z)
	    {
	    }

	    /// \brief Constructor
	    public: Quaternion(float4 _quat)
	    : w(_quat.w), x(_quat.x), y(_quat.y), z(_quat.z)
	    {
	    }


	    /// \brief Constructor from Euler angles in radians
	    public: Quaternion(const float &_roll, const float &_pitch,
	    		const float &_yaw)
	    {
		      float phi, the, psi;

		      phi = _roll / 2.0;
		      the = _pitch / 2.0;
		      psi = _yaw / 2.0;

		      this->w = cos(phi) * cos(the) * cos(psi) + sin(phi) * sin(the) * sin(psi);
		      this->x = sin(phi) * cos(the) * cos(psi) - cos(phi) * sin(the) * sin(psi);
		      this->y = cos(phi) * sin(the) * cos(psi) + sin(phi) * cos(the) * sin(psi);
		      this->z = cos(phi) * cos(the) * sin(psi) - sin(phi) * sin(the) * cos(psi);

		      this->Normalize();
	    }


	    /// \brief Set this quaternion from 4 floating numbers
	    public: void Set(float _w, float _x, float _y, float _z)
	    {
	    	this->w = _w;
	    	this->x = _x;
	    	this->y = _y;
	    	this->z = _z;
	    }


	    /// \brief Constructor from axis angle
	    public: void SetFromAxis(float _ax, float _ay, float _az, float _aa)
	    {
	      float l;

	      l = _ax * _ax + _ay * _ay + _az * _az;

	      if (l == 0.0)
	      {
	        this->w = 1;
	        this->x = 0;
	        this->y = 0;
	        this->z = 0;
	      }
	      else
	      {
	        _aa *= 0.5;
	        l = sin(_aa) / sqrt(l);
	        this->w = cos(_aa);
	        this->x = _ax * l;
	        this->y = _ay * l;
	        this->z = _az * l;
	      }

	      this->Normalize();
	    }



	    /// \brief Normalize the quaternion
	    public: void Normalize()
	    {
	      float s = 0;

	      s = sqrt(this->w * this->w + this->x * this->x + this->y * this->y +
	               this->z * this->z);

	      if (s == 0.0)
	      {
	        this->w = 1.0;
	        this->x = 0.0;
	        this->y = 0.0;
	        this->z = 0.0;
	      }
	      else
	      {
	        this->w /= s;
	        this->x /= s;
	        this->y /= s;
	        this->z /= s;
	      }
	    };


	    /// \brief Get the inverse of this quaternion
	    /// \return Inverse quarenion
	    public: inline Quaternion GetInverse() const
	            {
	              float s = 0;
	              Quaternion q(this->w, this->x, this->y, this->z);

	              // use s to test if quaternion is valid
	              s = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;

	              if (s == 0.0)
	              {
	                q.w = 1.0;
	                q.x = 0.0;
	                q.y = 0.0;
	                q.z = 0.0;
	              }
	              else
	              {
	                // deal with non-normalized quaternion
	                // div by s so q * qinv = identity
	                q.w =  q.w / s;
	                q.x = -q.x / s;
	                q.y = -q.y / s;
	                q.z = -q.z / s;
	              }
	              return q;
	            }


	    /// \brief Rotate a vector using the quaternion
	    public: inline float3 RotateVector(const float3 &_vec) const
	            {
	              Quaternion tmp(0.0, _vec.x, _vec.y, _vec.z);
	              tmp = (*this) * (tmp * this->GetInverse());
	              return make_float3(tmp.x, tmp.y, tmp.z);
	            }

	    /// \brief Rotate a vector using the quaternion
	    public: inline float4 RotateVector(const float4 &_vec) const
	            {
	              Quaternion tmp(0.0, _vec.x, _vec.y, _vec.z);
	              tmp = (*this) * (tmp * this->GetInverse());
	              return make_float4(tmp.x, tmp.y, tmp.z, 0);
	            }


	    /// \brief Multiplication operator
	    public: inline Quaternion operator*(const Quaternion &_q) const
	            {
	              return Quaternion(
	                  this->w*_q.w - this->x*_q.x - this->y*_q.y - this->z*_q.z,
	                  this->w*_q.x + this->x*_q.w + this->y*_q.z - this->z*_q.y,
	                  this->w*_q.y - this->x*_q.z + this->y*_q.w + this->z*_q.x,
	                  this->w*_q.z + this->x*_q.y - this->y*_q.x + this->z*_q.w);
	            }


	    /// \return quaternion multiplied by _f
	    public: Quaternion operator*(const float &_f) const
	    {
	    	return Quaternion(this->w*_f, this->x*_f, this->y*_f, this->z*_f);
	    }


	    /// \brief Subtraction operator
	    public: Quaternion operator-(const Quaternion &qt) const
	    {
	      Quaternion result(this->w - qt.w, this->x - qt.x,
	                     this->y - qt.y, this->z - qt.z);
	      return result;
	    }


	    /// \brief Attributes of the quaternion
	    public: float w;

	    /// \brief Attributes of the quaternion
	    public: float x;

	    /// \brief Attributes of the quaternion
	    public: float y;

	    /// \brief Attributes of the quaternion
	    public: float z;
	};
}



#endif /* QUATERNION_H_ */
