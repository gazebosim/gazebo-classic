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
#ifndef MATRIX4_HH
#define MATRIX4_HH

#include <assert.h>
#include <iostream>

#include "math/Vector3.hh"
#include "math/Matrix3.hh"

namespace gazebo
{
  namespace math
  {
    class Quaternion;
    class Pose;

    /// \addtogroup gazebo_math
    /// \{

    /// \brief A 3x3 matrix class
    class Matrix4
    {
      /// \brief Constructor
      public: Matrix4();

      /// \brief Copy constructor
      /// \param _m Matrix to copy
      public: Matrix4(const Matrix4 &_m);

      /// \brief Constructor
      /// \param _v00 Row 0, Col 0 value
      /// \param _v01 Row 0, Col 1 value
      /// \param _v02 Row 0, Col 2 value
      /// \param _v03 Row 0, Col 3 value
      /// \param _v10 Row 1, Col 0 value
      /// \param _v11 Row 1, Col 1 value
      /// \param _v12 Row 1, Col 2 value
      /// \param _v13 Row 1, Col 3 value
      /// \param _v20 Row 2, Col 0 value
      /// \param _v21 Row 2, Col 1 value
      /// \param _v22 Row 2, Col 2 value
      /// \param _v23 Row 2, Col 3 value
      /// \param _v30 Row 3, Col 0 value
      /// \param _v31 Row 3, Col 1 value
      /// \param _v32 Row 3, Col 2 value
      /// \param _v33 Row 3, Col 3 value
      public: Matrix4(double _v00, double _v01, double _v02, double _v03,
                      double _v10, double _v11, double _v12, double _v13,
                      double _v20, double _v21, double _v22, double _v23,
                      double _v30, double _v31, double _v32, double _v33);

      /// \brief Destructor
      public: virtual ~Matrix4();

      /// \brief Set
      /// \param _v00 Row 0, Col 0 value
      /// \param _v01 Row 0, Col 1 value
      /// \param _v02 Row 0, Col 2 value
      /// \param _v03 Row 0, Col 3 value
      /// \param _v10 Row 1, Col 0 value
      /// \param _v11 Row 1, Col 1 value
      /// \param _v12 Row 1, Col 2 value
      /// \param _v13 Row 1, Col 3 value
      /// \param _v20 Row 2, Col 0 value
      /// \param _v21 Row 2, Col 1 value
      /// \param _v22 Row 2, Col 2 value
      /// \param _v23 Row 2, Col 3 value
      /// \param _v30 Row 3, Col 0 value
      /// \param _v31 Row 3, Col 1 value
      /// \param _v32 Row 3, Col 2 value
      /// \param _v33 Row 3, Col 3 value
      public: void Set(double _v00, double _v01, double _v02, double _v03,
                       double _v10, double _v11, double _v12, double _v13,
                       double _v20, double _v21, double _v22, double _v23,
                       double _v30, double _v31, double _v32, double _v33);



      /// \brief Set the translational values [ (0, 3) (1, 3) (2, 3) ]
      /// \param _t Values to set
      public: void SetTranslate(const Vector3 &_t);

      /// \brief Get the translational values as a Vector3
      public: Vector3 GetTranslation() const;

      /// \brief Get the rotation as a quaternion
      public: Quaternion GetRotation() const;

      /// \brief Get the rotation as a Euler angles
      public: Vector3 GetEulerRotation(unsigned int solution_number = 1) const;

      /// \brief Get the transformation as math::Pose
      public: math::Pose GetAsPose() const;

      /// \brief Set the scale
      /// \param _s scale
      public: void SetScale(const Vector3 &_s);

      /// \brief Return true if the matrix is affine
      /// \return True if the matrix is affine
      public: bool IsAffine() const;

      /// \brief Perform an affine transformation
      /// \param _v Vector3 value for the transformation
      /// \return The result of the transformation
      public: Vector3 TransformAffine(const Vector3 &_v) const;

      /// \brief Return the inverse matrix
      public: Matrix4 Inverse() const;

      /// \brief Equal operator. this = _mat
      /// \param _mat Incoming matrix
      /// \return The resulting matrix
      public: Matrix4 &operator =(const Matrix4 &_mat);

      /// \brief Equal operator for 3x3 matrix
      /// \param _mat Incoming matrix
      /// \return The resulting matrix
      public: const Matrix4 & operator =(const Matrix3 &_mat);

      /// \brief Multiplication operator
      /// \param _mat Incoming matrix
      /// \return This matrix * _mat
      public: Matrix4 operator*(const Matrix4 &_mat) const;

      /// \brief Multiplication operator
      /// \param _mat Incoming matrix
      /// \return This matrix * _mat
      public: Matrix4 operator*(const Matrix3 &_mat) const;


      /// \brief Multiplication operator
      /// \param _vec Vector3
      /// \return Resulting vector from multiplication
      public: Vector3 operator*(const Vector3 &_vec) const;

      public: inline double *operator[](size_t _row)
              {
                assert(_row < 4);
                return this->m[_row];
              }

       public: inline const double *operator[](size_t _row) const
              {
                assert(_row < 4);
                return this->m[_row];
              }

      /// \brief Equality test operatoer
      /// \param _m Matrix3 to test
      /// \return True if equal
      public: bool operator==(const Matrix4 &_m) const;

      /// \brief Output operator
      /// \param _out Output stream
      /// \param _m Matrix to output
      public: friend std::ostream &operator<<(std::ostream &_out,
                                               const gazebo::math::Matrix4 &_m)
            {
              for (int i = 0; i < 4; i++)
              {
                for (int j = 0; j < 4; j++)
                {
                  _out << (fabs(_m.m[i][j]) < 1e-6 ? 0 : _m.m[i][j]) << " ";
                }
                _out << "\n";
              }

              return _out;
            }

      /// \brief Identity matrix
      public: static const Matrix4 IDENTITY;

      /// \brief Zero matrix
      public: static const Matrix4 ZERO;

      /// \brief The 4x4 matrix
      protected: double m[4][4];
    };
    /// \}
  }
}
#endif



