/*
-----------------------------------------------------------------------------
This source file is part of OGRE
    (Object-oriented Graphics Rendering Engine)
For the latest info, see http://www.ogre3d.org/

Copyright ( _c) 2000-2009 Torus Knot Software Ltd

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the _"Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
-----------------------------------------------------------------------------
*/

#include "math/Helpers.hh"
#include "math/Vector4.hh"
#include "math/Spline.hh"

using namespace gazebo;
using namespace math;

///////////////////////////////////////////////////////////
Spline::Spline()
{
  // Set up matrix
  // Hermite polynomial
  this->coeffs[0][0] = 2;
  this->coeffs[0][1] = -2;
  this->coeffs[0][2] = 1;
  this->coeffs[0][3] = 1;
  this->coeffs[1][0] = -3;
  this->coeffs[1][1] = 3;
  this->coeffs[1][2] = -2;
  this->coeffs[1][3] = -1;
  this->coeffs[2][0] = 0;
  this->coeffs[2][1] = 0;
  this->coeffs[2][2] = 1;
  this->coeffs[2][3] = 0;
  this->coeffs[3][0] = 1;
  this->coeffs[3][1] = 0;
  this->coeffs[3][2] = 0;
  this->coeffs[3][3] = 0;

  this->autoCalc = true;
}

///////////////////////////////////////////////////////////
Spline::~Spline()
{
}

///////////////////////////////////////////////////////////
void Spline::AddPoint(const Vector3 &_p)
{
  this->points.push_back(_p);
  if (this->autoCalc)
    this->RecalcTangents();
}

///////////////////////////////////////////////////////////
Vector3 Spline::Interpolate(double _t) const
{
  // Currently assumes points are evenly spaced, will cause velocity
  // change where this is not the case
  // TODO: base on arclength?

  // Work out which segment this is in
  double fSeg = _t * (this->points.size() - 1);
  unsigned int segIdx = (unsigned int)fSeg;
  // Apportion t
  _t = fSeg - segIdx;

  return this->Interpolate(segIdx, _t);
}

///////////////////////////////////////////////////////////
Vector3 Spline::Interpolate(unsigned int _fromIndex, double _t) const
{
  // Bounds check
  assert (_fromIndex < this->points.size() &&
      "_fromIndex out of bounds");

  if ((_fromIndex + 1) == this->points.size())
  {
    // Duff request, cannot blend to nothing
    // Just return source
    return this->points[_fromIndex];
  }

  // Fast special cases
  if (equal(_t, 0.0))
    return this->points[_fromIndex];
  else if (equal(_t, 1.0))
    return this->points[_fromIndex + 1];

  // double interpolation
  // Form a vector of powers of t
  double t2, t3;
  t2 = _t * _t;
  t3 = t2 * _t;
  Vector4 powers(t3, t2, _t, 1);


  // Algorithm is ret = powers * this->coeffs * Matrix4(point1,
  // point2, tangent1, tangent2)
  const Vector3 &point1 = this->points[_fromIndex];
  const Vector3 &point2 = this->points[_fromIndex+1];
  const Vector3 &tan1 = this->tangents[_fromIndex];
  const Vector3 &tan2 = this->tangents[_fromIndex+1];
  Matrix4 pt;

  pt[0][0] = point1.x;
  pt[0][1] = point1.y;
  pt[0][2] = point1.z;
  pt[0][3] = 1.0f;
  pt[1][0] = point2.x;
  pt[1][1] = point2.y;
  pt[1][2] = point2.z;
  pt[1][3] = 1.0f;
  pt[2][0] = tan1.x;
  pt[2][1] = tan1.y;
  pt[2][2] = tan1.z;
  pt[2][3] = 1.0f;
  pt[3][0] = tan2.x;
  pt[3][1] = tan2.y;
  pt[3][2] = tan2.z;
  pt[3][3] = 1.0f;

  Vector4 ret = powers * this->coeffs * pt;

  return Vector3(ret.x, ret.y, ret.z);
}

///////////////////////////////////////////////////////////
void Spline::RecalcTangents()
{
  // Catmull-Rom approach
  //
  // tangent[i] = 0.5 * (point[i+1] - point[i-1])
  //
  // Assume endpoint tangents are parallel with line with neighbour

  size_t i, numPoints;
  bool isClosed;

  numPoints = this->points.size();
  if (numPoints < 2)
  {
    // Can't do anything yet
    return;
  }

  // Closed or open?
  if (this->points[0] == this->points[numPoints-1])
    isClosed = true;
  else
    isClosed = false;

  this->tangents.resize(numPoints);

  for (i = 0; i < numPoints; ++i)
  {
    if (i == 0)
    {
      // Special case start
      if (isClosed)
      {
        // Use nuthis->points-2 since nuthis->points-1 is the last
        // point and == [0]
        this->tangents[i] = (this->points[1] - this->points[numPoints-2]) * 0.5;
      }
      else
      {
        this->tangents[i] = (this->points[1] - this->points[0]) * 0.5;
      }
    }
    else if (i == numPoints-1)
    {
      // Special case end
      if (isClosed)
      {
        // Use same tangent as already calculated for [0]
        this->tangents[i] = this->tangents[0];
      }
      else
      {
        this->tangents[i] = (this->points[i] - this->points[i-1]) * 0.5;
      }
    }
    else
    {
      this->tangents[i] = (this->points[i+1] - this->points[i-1]) * 0.5;
    }
  }
}

///////////////////////////////////////////////////////////
const Vector3 &Spline::GetPoint(unsigned int _index) const
{
  assert (_index < this->points.size() && "Point index is out of bounds!!");
  return this->points[_index];
}

///////////////////////////////////////////////////////////
unsigned int Spline::GetNumPoints() const
{
  return this->points.size();
}

///////////////////////////////////////////////////////////
void Spline::Clear()
{
  this->points.clear();
  this->tangents.clear();
}

///////////////////////////////////////////////////////////
void Spline::UpdatePoint(unsigned int _index, const Vector3 &_value)
{
  assert (_index < this->points.size() && "Point index is out of bounds!!");

  this->points[_index] = _value;
  if (this->autoCalc)
    this->RecalcTangents();
}

///////////////////////////////////////////////////////////
void Spline::SetAutoCalculate(bool _autoCalc)
{
  this->autoCalc = _autoCalc;
}


