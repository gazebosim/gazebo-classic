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
