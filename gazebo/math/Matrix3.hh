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
#ifndef MATRIX3_HH
#define MATRIX3_HH

#include <assert.h>

#include "math/Vector3.hh"

namespace gazebo
{
  namespace math
  {
    /// \addtogroup gazebo_math
    /// \{

    /// \brief A 3x3 matrix class
    class Matrix3
    {
      /// \brief Constructor
      public: Matrix3();

      /// \brief Copy constructor
      /// \param _m Matrix to copy
      public: Matrix3(const Matrix3 &_m);

      /// \brief Constructor
      /// \param _v00 Row 0, Col 0 value
      /// \param _v01 Row 0, Col 1 value
      /// \param _v02 Row 0, Col 2 value
      /// \param _v10 Row 1, Col 0 value
      /// \param _v11 Row 1, Col 1 value
      /// \param _v12 Row 1, Col 2 value
      /// \param _v20 Row 2, Col 0 value
      /// \param _v21 Row 2, Col 1 value
      /// \param _v22 Row 2, Col 2 value
      public: Matrix3(double _v00, double _v01, double _v02,
                      double _v10, double _v11, double _v12,
                      double _v20, double _v21, double _v22);

      /// \brief Desctructor
      public: virtual ~Matrix3();

      /// \brief Set the matrix from three axis
      /// \param _xAxis The x axis
      /// \param _yAxis The y axis
      /// \param _zAxis The z axis
      public: void SetFromAxes(const Vector3 &_xAxis,
                               const Vector3 &_yAxis,
                               const Vector3 &_zAxis);


      /// \brief Set the matrix from an axis and angle
      public: void SetFromAxis(const Vector3 &_axis, double _angle);

      /// \brief Set a column
      /// \param _c The colum index (0, 1, 2)
      /// \param _v The value to set in each row of the column
      public: void SetCol(unsigned int _c, const Vector3 &_v);

      /// \brief Equality test operatoer
      /// \param _m Matrix3 to test
      /// \return True if equal
      public: bool operator==(const Matrix3 &_m) const;

      public: inline const double *operator[](size_t _row) const
              {
                assert(_row < 3);
                return this->m[_row];
              }

      public: inline double *operator[](size_t _row)
              {
                assert(_row < 3);
                return this->m[_row];
              }


      /// \brief Output operator
      /// \param _out Output stream
      /// \param _m Matrix to output
      public: friend std::ostream &operator<<(std::ostream &_out,
                                               const gazebo::math::Matrix3 &_m)
            {
              for (int i = 0; i < 3; i++)
              {
                for (int j = 0; j < 3; j++)
                {
                  _out << _m.m[i][j] << " ";
                }
                _out << "\n";
              }

              return _out;
            }

      /// \brief the 3x3 matrix
      protected: double m[3][3];
      friend class Matrix4;
    };
    /// \}
  }
}
#endif



