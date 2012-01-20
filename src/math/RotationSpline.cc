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
#include "RotationSpline.hh"

using namespace gazebo;
using namespace math;

RotationSpline::RotationSpline()
: autoCalc(true)
{
}

RotationSpline::~RotationSpline()
{
}

void RotationSpline::AddPoint(const Quaternion &_p)
{
  this->points.push_back(_p);
  if (this->autoCalc)
    this->RecalcTangents();
}

Quaternion RotationSpline::Interpolate(double _t, bool _useShortestPath)
{
  // Work out which segment this is in
  double fSeg = _t * (this->points.size() - 1);
  unsigned int segIdx = (unsigned int)fSeg;

  // Apportion t
  _t = fSeg - segIdx;

  return this->Interpolate(segIdx, _t, _useShortestPath);
}

Quaternion RotationSpline::Interpolate(unsigned int _fromIndex, double _t,
                                       bool _useShortestPath)
{
  // Bounds check
  assert (_fromIndex < this->points.size() && "fromIndex out of bounds");

  if ((_fromIndex + 1) == this->points.size())
  {
    // Duff request, cannot blend to nothing
    // Just return source
    return this->points[_fromIndex];
  }

  // Fast special cases
  if (math::equal(_t, 0.0))
    return this->points[_fromIndex];
  else if (math::equal(_t, 1.0))
    return this->points[_fromIndex + 1];

  // double interpolation
  // Use squad using tangents we've already set up
  Quaternion &p = this->points[_fromIndex];
  Quaternion &q = this->points[_fromIndex+1];
  Quaternion &a = this->tangents[_fromIndex];
  Quaternion &b = this->tangents[_fromIndex+1];

  // NB interpolate to nearest rotation
  return Quaternion::Squad(_t, p, a, b, q, _useShortestPath);
}

void RotationSpline::RecalcTangents()
{
  // ShoeMake (1987) approach
  // Just like Catmull-Rom really, just more gnarly
  // And no, I don't understand how to derive this!
  //
  // let p = point[i], pInv = p.Inverse
  // tangent[i] = p * exp(-0.25 *
  // (log(pInv * point[i+1]) + log(pInv * point[i-1])))
  //
  // Assume endpoint tangents are parallel with line with neighbour

  unsigned int i, numPoints;
  bool isClosed;

  numPoints = this->points.size();

  if (numPoints < 2)
  {
    // Can't do anything yet
    return;
  }

  this->tangents.resize(numPoints);

  if (this->points[0] == this->points[numPoints-1])
    isClosed = true;
  else
    isClosed = false;

  Quaternion invp, part1, part2, preExp;
  for (i = 0; i < numPoints; ++i)
  {
    Quaternion &p = this->points[i];
    invp = p.GetInverse();

    if (i == 0)
    {
      // special case start
      part1 = (invp * this->points[i+1]).GetLog();
      if (isClosed)
      {
        // Use numPoints-2 since numPoints-1 == end == start == this one
        part2 = (invp * this->points[numPoints-2]).GetLog();
      }
      else
      {
        part2 = (invp * p).GetLog();
      }
    }
    else if (i == numPoints-1)
    {
      // special case end
      if (isClosed)
      {
        // Wrap to [1] (not [0], this is the same as end == this one)
        part1 = (invp * this->points[1]).GetLog();
      }
      else
      {
        part1 = (invp * p).GetLog();
      }
      part2 = (invp * this->points[i-1]).GetLog();
    }
    else
    {
      part1 = (invp * this->points[i+1]).GetLog();
      part2 = (invp * this->points[i-1]).GetLog();
    }

    preExp = (part1 + part2) * -0.25;
    this->tangents[i] = p * preExp.GetExp();
  }
}

const Quaternion& RotationSpline::GetPoint(unsigned int _index) const
{
  assert (_index < this->points.size() && "Point index is out of bounds!!");

  return this->points[_index];
}

unsigned int RotationSpline::GetNumPoints() const
{
  return this->points.size();
}

void RotationSpline::Clear()
{
  this->points.clear();
  this->tangents.clear();
}

void RotationSpline::UpdatePoint(unsigned int _index,
                                 const Quaternion &_value)
{
  assert (_index < this->points.size() && "Point index is out of bounds!!");

  this->points[_index] = _value;
  if (this->autoCalc)
    this->RecalcTangents();
}

void RotationSpline::SetAutoCalculate(bool _autoCalc)
{
  this->autoCalc = _autoCalc;
}


