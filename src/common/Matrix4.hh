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
#ifndef MATRIX4_HH
#define MATRIX4_HH

#include <iostream>

#include "common/Vector3.hh"
#include "common/Matrix3.hh"

namespace gazebo
{
	namespace common
{
  class Matrix4
  {
    public: Matrix4();
    public: Matrix4(const Matrix4 &m);
    public: Matrix4(double v00, double v01, double v02, double v03,
                    double v10, double v11, double v12, double v13,
                    double v20, double v21, double v22, double v23,
                    double v30, double v31, double v32, double v33);

    public: virtual ~Matrix4();

    public: void SetTrans(const Vector3 &t);

    public: bool IsAffine() const;
    public: Vector3 TransformAffine( const Vector3 &v ) const;

    public: const Matrix4 &operator=( const Matrix4 &mat );
    public: const Matrix4 & operator=( const Matrix3 &mat );

    public: Matrix4 operator*(const Matrix4 &mat);

    public: friend std::ostream &operator<<( std::ostream &out, const gazebo::common::Matrix4 &m )
          {
            for (int i=0; i < 4; i++)
            {
              for (int j=0; j < 4; j++)
              {
                out << m.m[i][j] << " ";
              }
              out << "\n";
            }

            return out;
          }

    public: static const Matrix4 IDENTITY;
    public: static const Matrix4 ZERO;


    protected: double m[4][4];
  };
}
}
#endif
