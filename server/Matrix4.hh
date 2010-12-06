#ifndef MATRIX4_HH
#define MATRIX4_HH

#include <iostream>

#include "Vector3.hh"
#include "Matrix3.hh"

namespace gazebo
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
    public: void operator=( const Matrix3 &mat );

    public: Matrix4 operator*(const Matrix4 &mat);

  public: friend std::ostream &operator<<( std::ostream &out, const gazebo::Matrix4 &m )
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
#endif
