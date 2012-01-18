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
#include <string.h>

#include "common/Exception.hh"
#include "math/Matrix4.hh"

using namespace gazebo;
using namespace math;

const Matrix4 Matrix4::IDENTITY(
       1.0, 0.0, 0.0, 0.0,
       0.0, 1.0, 0.0, 0.0,
       0.0, 0.0, 1.0, 0.0,
       0.0, 0.0, 0.0, 1.0);


const Matrix4 Matrix4::ZERO(
       0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0);


//////////////////////////////////////////////////
// Default constructor
Matrix4::Matrix4()
{
  memset(this->m, 0, sizeof(this->m[0][0])*16);
}

//////////////////////////////////////////////////
// Copy constructor
Matrix4::Matrix4(const Matrix4 &_m)
{
  memcpy(this->m, _m.m, sizeof(this->m[0][0])*16);
}

//////////////////////////////////////////////////
// Constructor
Matrix4::Matrix4(double _v00, double _v01, double _v02, double _v03,
                 double _v10, double _v11, double _v12, double _v13,
                 double _v20, double _v21, double _v22, double _v23,
                 double _v30, double _v31, double _v32, double _v33)
{
  this->Set(_v00, _v01, _v02, _v03,
            _v10, _v11, _v12, _v13,
            _v20, _v21, _v22, _v23,
            _v30, _v31, _v32, _v33);
}

//////////////////////////////////////////////////
// Destructor
Matrix4::~Matrix4()
{
}

//////////////////////////////////////////////////
// Constructor
void Matrix4::Set(double _v00, double _v01, double _v02, double _v03,
                  double _v10, double _v11, double _v12, double _v13,
                  double _v20, double _v21, double _v22, double _v23,
                  double _v30, double _v31, double _v32, double _v33)
{
  this->m[0][0] = _v00;
  this->m[0][1] = _v01;
  this->m[0][2] = _v02;
  this->m[0][3] = _v03;

  this->m[1][0] = _v10;
  this->m[1][1] = _v11;
  this->m[1][2] = _v12;
  this->m[1][3] = _v13;

  this->m[2][0] = _v20;
  this->m[2][1] = _v21;
  this->m[2][2] = _v22;
  this->m[2][3] = _v23;

  this->m[3][0] = _v30;
  this->m[3][1] = _v31;
  this->m[3][2] = _v32;
  this->m[3][3] = _v33;
}

//////////////////////////////////////////////////
// Set translation
void Matrix4::SetTranslate(const Vector3 &_t)
{
  this->m[0][3] = _t.x;
  this->m[1][3] = _t.y;
  this->m[2][3] = _t.z;
}

//////////////////////////////////////////////////
/// Set the scale
void Matrix4::SetScale(const Vector3 &_s)
{
  this->m[0][0] = _s.x;
  this->m[1][1] = _s.y;
  this->m[2][2] = _s.z;
  this->m[3][3] = 1.0;
}


//////////////////////////////////////////////////
// Equality operator
Matrix4 &Matrix4::operator =(const Matrix4 &_mat)
{
  memcpy(this->m, _mat.m, sizeof(this->m[0][0])*16);
  return *this;
}

//////////////////////////////////////////////////
// Equality operator
const Matrix4 &Matrix4::operator =(const Matrix3 &mat)
{
  this->m[0][0] = mat.m[0][0];
  this->m[0][1] = mat.m[0][1];
  this->m[0][2] = mat.m[0][2];

  this->m[1][0] = mat.m[1][0];
  this->m[1][1] = mat.m[1][1];
  this->m[1][2] = mat.m[1][2];

  this->m[2][0] = mat.m[2][0];
  this->m[2][1] = mat.m[2][1];
  this->m[2][2] = mat.m[2][2];

  return *this;
}


//////////////////////////////////////////////////
// Mult operator
Matrix4 Matrix4::operator*(const Matrix3 &m2) const
{
  Matrix4 r;
  r = *this;

  r.m[0][0] = m[0][0]*m2.m[0][0] + m[0][1]*m2.m[1][0] + m[0][2] * m2.m[2][0];
  r.m[0][1] = m[0][0]*m2.m[0][1] + m[0][1]*m2.m[1][1] + m[0][2] * m2.m[2][1];
  r.m[0][2] = m[0][0]*m2.m[0][2] + m[0][1]*m2.m[1][2] + m[0][2] * m2.m[2][2];

  r.m[1][0] = m[1][0]*m2.m[0][0] + m[1][1]*m2.m[1][0] + m[1][2] * m2.m[2][0];
  r.m[1][1] = m[1][0]*m2.m[0][1] + m[1][1]*m2.m[1][1] + m[1][2] * m2.m[2][1];
  r.m[1][2] = m[1][0]*m2.m[0][2] + m[1][1]*m2.m[1][2] + m[1][2] * m2.m[2][2];

  r.m[2][0] = m[2][0]*m2.m[0][0] + m[2][1]*m2.m[1][0] + m[2][2] * m2.m[2][0];
  r.m[2][1] = m[2][0]*m2.m[0][1] + m[2][1]*m2.m[1][1] + m[2][2] * m2.m[2][1];
  r.m[2][2] = m[2][0]*m2.m[0][2] + m[2][1]*m2.m[1][2] + m[2][2] * m2.m[2][2];

  return r;
}

//////////////////////////////////////////////////
// Mult operator
Matrix4 Matrix4::operator*(const Matrix4 &m2) const
{
  Matrix4 r;

  r.m[0][0] = this->m[0][0] * m2.m[0][0] +
              this->m[0][1] * m2.m[1][0] +
              this->m[0][2] * m2.m[2][0] +
              this->m[0][3] * m2.m[3][0];

  r.m[0][1] = this->m[0][0] * m2.m[0][1] +
              this->m[0][1] * m2.m[1][1] +
              this->m[0][2] * m2.m[2][1] +
              this->m[0][3] * m2.m[3][1];

  r.m[0][2] = this->m[0][0] * m2.m[0][2] +
              this->m[0][1] * m2.m[1][2] +
              this->m[0][2] * m2.m[2][2] +
              this->m[0][3] * m2.m[3][2];

  r.m[0][3] = this->m[0][0] * m2.m[0][3] +
              this->m[0][1] * m2.m[1][3] +
              this->m[0][2] * m2.m[2][3] +
              this->m[0][3] * m2.m[3][3];

  r.m[1][0] = this->m[1][0] * m2.m[0][0] +
              this->m[1][1] * m2.m[1][0] +
              this->m[1][2] * m2.m[2][0] +
              this->m[1][3] * m2.m[3][0];

  r.m[1][1] = this->m[1][0] * m2.m[0][1] +
              this->m[1][1] * m2.m[1][1] +
              this->m[1][2] * m2.m[2][1] +
              this->m[1][3] * m2.m[3][1];

  r.m[1][2] = this->m[1][0] * m2.m[0][2] +
              this->m[1][1] * m2.m[1][2] +
              this->m[1][2] * m2.m[2][2] +
              this->m[1][3] * m2.m[3][2];

  r.m[1][3] = this->m[1][0] * m2.m[0][3] +
              this->m[1][1] * m2.m[1][3] +
              this->m[1][2] * m2.m[2][3] +
              this->m[1][3] * m2.m[3][3];

  r.m[2][0] = this->m[2][0] * m2.m[0][0] +
              this->m[2][1] * m2.m[1][0] +
              this->m[2][2] * m2.m[2][0] +
              this->m[2][3] * m2.m[3][0];

  r.m[2][1] = this->m[2][0] * m2.m[0][1] +
              this->m[2][1] * m2.m[1][1] +
              this->m[2][2] * m2.m[2][1] +
              this->m[2][3] * m2.m[3][1];

  r.m[2][2] = this->m[2][0] * m2.m[0][2] +
              this->m[2][1] * m2.m[1][2] +
              this->m[2][2] * m2.m[2][2] +
              this->m[2][3] * m2.m[3][2];

  r.m[2][3] = this->m[2][0] * m2.m[0][3] +
              this->m[2][1] * m2.m[1][3] +
              this->m[2][2] * m2.m[2][3] +
              this->m[2][3] * m2.m[3][3];

  r.m[3][0] = this->m[3][0] * m2.m[0][0] +
              this->m[3][1] * m2.m[1][0] +
              this->m[3][2] * m2.m[2][0] +
              this->m[3][3] * m2.m[3][0];

  r.m[3][1] = this->m[3][0] * m2.m[0][1] +
              this->m[3][1] * m2.m[1][1] +
              this->m[3][2] * m2.m[2][1] +
              this->m[3][3] * m2.m[3][1];

  r.m[3][2] = this->m[3][0] * m2.m[0][2] +
              this->m[3][1] * m2.m[1][2] +
              this->m[3][2] * m2.m[2][2] +
              this->m[3][3] * m2.m[3][2];

  r.m[3][3] = this->m[3][0] * m2.m[0][3] +
              this->m[3][1] * m2.m[1][3] +
              this->m[3][2] * m2.m[2][3] +
              this->m[3][3] * m2.m[3][3];

  return r;
}

//////////////////////////////////////////////////
/// Multiplication operator
Vector3 Matrix4::operator*(const Vector3 &_vec) const
{
  Vector3 result;
  result.x = this->m[0][0]*_vec.x + this->m[0][1]*_vec.y +
             this->m[0][2]*_vec.z + this->m[0][3];
  result.y = this->m[1][0]*_vec.x + this->m[1][1]*_vec.y +
             this->m[1][2]*_vec.z + this->m[1][3];
  result.z = this->m[2][0]*_vec.x + this->m[2][1]*_vec.y +
             this->m[2][2]*_vec.z + this->m[2][3];
  return result;
}

//////////////////////////////////////////////////
// Return true if affine
bool Matrix4::IsAffine() const
{
  return this->m[3][0] == 0 && this->m[3][1] == 0 &&
         this->m[3][2] == 0 && this->m[3][3] == 1;
}

//////////////////////////////////////////////////
// Affine transform
Vector3 Matrix4::TransformAffine(const Vector3 &_v) const
{
  if (!this->IsAffine())
  {
    gzthrow("Not and affine matrix");
  }

  return Vector3(this->m[0][0]*_v.x + this->m[0][1]*_v.y +
                 this->m[0][2]*_v.z + this->m[0][3],
                 this->m[1][0]*_v.x + this->m[1][1]*_v.y +
                 this->m[1][2]*_v.z + this->m[1][3],
                 this->m[2][0]*_v.x + this->m[2][1]*_v.y +
                 this->m[2][2]*_v.z + this->m[2][3]);
}

