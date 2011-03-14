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
#ifndef MATRIX3_HH
#define MATRIX3_HH

#include "Vector3.hh"

namespace gazebo
{
  class Matrix3
  {
    public: Matrix3();
    public: Matrix3(const Matrix3 &m);
    public: Matrix3(double v00, double v01, double v02,
                    double v10, double v11, double v12,
                    double v20, double v21, double v22);

    public: virtual ~Matrix3();

    public: void SetFromAxes(const Vector3 &xAxis, const Vector3 &yAxis, 
                             const Vector3 &zAxis);

    /// Set a column
    public: void SetCol(unsigned int i, const Vector3 &v);

    protected: double m[3][3];
    friend class Matrix4;
  };
}
#endif
