/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */

#ifndef URDF_POSE_H
#define URDF_POSE_H

#include <stdio.h>
#include <string>
#include <vector>
#include <math.h>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

namespace sdf{

class Vector3
{
  public: Vector3(double _x,double _y, double _z) 
          {this->x=_x;this->y=_y;this->z=_z;};
  
  public: Vector3() {this->Clear();};
  public: double x;
  public: double y;
  public: double z;

  public: void Clear() {this->x=this->y=this->z=0.0;};
  public: bool Init(const std::string &_vectorStr)
          {
            this->Clear();
            std::vector<std::string> pieces;
            std::vector<double> xyz;
            boost::split( pieces, _vectorStr, boost::is_any_of(" "));
            for (unsigned int i = 0; i < pieces.size(); ++i)
            {
              if (pieces[i] != "")
              {
                try
                {
                  xyz.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
                }
                catch (boost::bad_lexical_cast &e)
                {
                  printf("Vector3 xyz element (%s) is not a valid float",pieces[i].c_str());
                  return false;
                }
              }
            }

            if (xyz.size() != 3) 
            {
              printf("Vector contains %i elements instead of 3 elements", (int)xyz.size()); 
              return false;
            }

            this->x = xyz[0];
            this->y = xyz[1];
            this->z = xyz[2];

            return true;
          };

  public: Vector3 operator+(Vector3 _vec)
  {
    return Vector3(this->x+_vec.x,this->y+_vec.y,this->z+_vec.z);
  };

  friend std::ostream &operator<<(std::ostream &out, const Vector3 &vec)
  {
    out << vec.x << " " << vec.y << " " << vec.z;
    return out;
  }
};

class Rotation
{
  public: Rotation(double _x,double _y, double _z, double _w) 
        {this->x=_x; this->y=_y; this->z=_z; this->w=_w;};

  public: Rotation() {this->Clear();};

  public: void GetQuaternion(double &_quatX,double &_quatY,
                             double &_quatZ, double &_quatW) const
  {
    _quatX = this->x;
    _quatY = this->y;
    _quatZ = this->z;
    _quatW = this->w;
  };

  void GetRPY(double &_roll,double &_pitch,double &_yaw) const
  {
    double sqw;
    double sqx;
    double sqy;
    double sqz;

    sqx = this->x * this->x;
    sqy = this->y * this->y;
    sqz = this->z * this->z;
    sqw = this->w * this->w;

    _roll  = atan2(2 * (this->y*this->z + this->w*this->x), sqw - sqx - sqy + sqz);
    double sarg = -2 * (this->x*this->z - this->w*this->y);
    _pitch = sarg <= -1.0 ? -0.5*M_PI : (sarg >= 1.0 ? 0.5*M_PI : asin(sarg));
    _yaw   = atan2(2 * (this->x*this->y + this->w*this->z), sqw + sqx - sqy - sqz);
  };

  void SetFromQuaternion(double _quatX,double _quatY,double _quatZ,double _quatW)
  {
    this->x = _quatX;
    this->y = _quatY;
    this->z = _quatZ;
    this->w = _quatW;
    this->Normalize();
  };

  void SetFromRPY(double _roll, double _pitch, double _yaw)
  {
    double phi, the, psi;

    phi = _roll / 2.0;
    the = _pitch / 2.0;
    psi = _yaw / 2.0;

    this->x = sin(phi) * cos(the) * cos(psi) - cos(phi) * sin(the) * sin(psi);
    this->y = cos(phi) * sin(the) * cos(psi) + sin(phi) * cos(the) * sin(psi);
    this->z = cos(phi) * cos(the) * sin(psi) - sin(phi) * sin(the) * cos(psi);
    this->w = cos(phi) * cos(the) * cos(psi) + sin(phi) * sin(the) * sin(psi);

    this->Normalize();
  };


  bool Init(const std::string &_rotationStr)
  {
    this->Clear();

    Vector3 rpy;
    
    if (!rpy.Init(_rotationStr))
      return false;
    else
    {
      this->SetFromRPY(rpy.x,rpy.y,rpy.z);
      return true;
    }
      
  };

  void Clear() { this->x=this->y=this->z=0.0;this->w=1.0; }

  void Normalize()
  {
    double s = sqrt(this->x * this->x +
                    this->y * this->y +
                    this->z * this->z +
                    this->w * this->w);
    if (s == 0.0)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
      this->w = 1.0;
    }
    else
    {
      this->x /= s;
      this->y /= s;
      this->z /= s;
      this->w /= s;
    }
  };

  // Multiplication operator (copied from gazebo)
  Rotation operator*( const Rotation &_qt ) const
  {
    Rotation c;

    c.x = this->w * _qt.x + this->x * _qt.w + this->y * _qt.z - this->z * _qt.y;
    c.y = this->w * _qt.y - this->x * _qt.z + this->y * _qt.w + this->z * _qt.x;
    c.z = this->w * _qt.z + this->x * _qt.y - this->y * _qt.x + this->z * _qt.w;
    c.w = this->w * _qt.w - this->x * _qt.x - this->y * _qt.y - this->z * _qt.z;

    return c;
  };

  /// Rotate a vector using the quaternion
  Vector3 operator*(Vector3 _vec) const
  {
    Rotation tmp;
    Vector3 result;

    tmp.w = 0.0;
    tmp.x = _vec.x;
    tmp.y = _vec.y;
    tmp.z = _vec.z;

    tmp = (*this) * (tmp * this->GetInverse());

    result.x = tmp.x;
    result.y = tmp.y;
    result.z = tmp.z;

    return result;
  };

  // Get the inverse of this quaternion
  Rotation GetInverse() const 
  {
    Rotation q;

    double norm = this->w*this->w+this->x*this->x+this->y*this->y+this->z*this->z;

    if (norm > 0.0)
    {
      q.w = this->w / norm;
      q.x = -this->x / norm;
      q.y = -this->y / norm;
      q.z = -this->z / norm;
    }

    return q;
  };

  friend std::ostream &operator<<(std::ostream &out, const Rotation &rot)
  {
    double r,p,y;
    rot.GetRPY(r,p,y);
    out << r << " " << p << " " << y;
    return out;
  }

  double x,y,z,w;
};

class Pose
{
  public: Pose() { this->Clear(); };

  public: Vector3  position;
  public: Rotation rotation;

  void Clear()
  {
    this->position.Clear();
    this->rotation.Clear();
  };


  friend std::ostream &operator<<(std::ostream &out, const Pose &pose)
  {
    out << "pos[" << pose.position << "] rot[" << pose.rotation << "]";
    return out;
  }
};

}

#endif
