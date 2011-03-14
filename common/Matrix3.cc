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

#include "GazeboError.hh"
#include "Matrix3.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Matrix3::Matrix3()
{
  memset(this->m, 0, sizeof(double)*9);
}

////////////////////////////////////////////////////////////////////////////////
// Constructor
Matrix3::Matrix3(const Matrix3 &m)
{
  memcpy(this->m, m.m, sizeof(double)*9);
}

////////////////////////////////////////////////////////////////////////////////
// Constructor
Matrix3::Matrix3(double v00, double v01, double v02,
                    double v10, double v11, double v12,
                    double v20, double v21, double v22)
{
  this->m[0][0] = v00;
  this->m[0][1] = v01;
  this->m[0][2] = v02;
  this->m[1][0] = v10;
  this->m[1][1] = v11;
  this->m[1][2] = v12;
  this->m[2][0] = v20;
  this->m[2][1] = v21;
  this->m[2][2] = v22;
}


////////////////////////////////////////////////////////////////////////////////
// Destructor
Matrix3::~Matrix3()
{
}

////////////////////////////////////////////////////////////////////////////////
// Set from axes
void Matrix3::SetFromAxes(const Vector3 &xAxis, const Vector3 &yAxis, 
                          const Vector3 &zAxis)
{
  this->SetCol(0, xAxis);
  this->SetCol(1, yAxis);
  this->SetCol(2, zAxis);
}


////////////////////////////////////////////////////////////////////////////////
/// Set a column
void Matrix3::SetCol(unsigned int i, const Vector3 &v)
{
  if (i >= 3)
    gzthrow("Invalid column number");

  m[0][i] = v.x;
  m[1][i] = v.y;
  m[2][i] = v.z;
}
