/*
-----------------------------------------------------------------------------
This source file is part of OGRE
    (Object-oriented Graphics Rendering Engine)
For the latest info, see http://www.ogre3d.org/

Copyright (_c) 2000-2009 Torus Knot Software Ltd

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
#ifndef _ROTATIONSPLINE_HH_
#define _ROTATIONSPLINE_HH_

#include <vector>
#include "math/Quaternion.hh"

namespace gazebo
{
  namespace math
  {
    /// \addtogroup gazebo_math
    /// \{

    /// \class RotationSpline RotationSpline.hh math/gzmath.hh
    /// \brief Spline for rotations
    class  RotationSpline
    {
        /// \brief Constructor. Sets the autoCalc to true
        public: RotationSpline();

        /// \brief Destructor. Nothing is done
        public: ~RotationSpline();

        /// \brief Adds a control point to the end of the spline.
        /// \param[in] _p control point
        public: void AddPoint(const Quaternion &_p);

        /// \brief Gets the detail of one of the control points of the spline.
        /// \param[in] _index the index of the control point.
        /// \remarks This point must already exist in the spline.
        /// \return a quaternion (out of bound index result in assertion)
        public: const Quaternion &GetPoint(unsigned int _index) const;

        /// \brief Gets the number of control points in the spline.
        /// \return the count
        public: unsigned int GetNumPoints() const;

        /// \brief Clears all the points in the spline.
        public: void Clear();

        /// \brief Updates a single point in the spline.
        /// \remarks This point must already exist in the spline.
        /// \param[in] _index index
        /// \param[in] _value the new control point value
        public: void UpdatePoint(unsigned int _index, const Quaternion &_value);

        /// \brief Returns an interpolated point based on a parametric
        ///        value over the whole series.
        /// \remarks Given a t value between 0 and 1 representing the
        ///          parametric distance along the whole length of the spline,
        ///          this method returns an interpolated point.
        /// \param[in] _t Parametric value.
        /// \param[in] _useShortestPath Defines if rotation should take the
        ///        shortest possible path
        /// \return the rotation
        public: Quaternion Interpolate(double _t, bool _useShortestPath = true);

        /// \brief Interpolates a single segment of the spline
        ///        given a parametric value.
        /// \param[in] _fromIndex The point index to treat as t = 0.
        ///        _fromIndex + 1 is deemed to be t = 1
        /// \param[in] _t Parametric value
        /// \param[in] _useShortestPath Defines if rotation should take the
        ///         shortest possible path
        /// \return the rotation
        public: Quaternion Interpolate(unsigned int _fromIndex, double _t,
            bool _useShortestPath = true);

        /// \brief Tells the spline whether it should automatically calculate
        ///        tangents on demand as points are added.
        /// \remarks The spline calculates tangents at each point automatically
        ///          based on the input points.  Normally it does this every
        ///          time a point changes. However, if you have a lot of points
        ///          to add in one go, you probably don't want to incur this
        ///          overhead and would prefer to defer the calculation until
        ///          you are finished setting all the points. You can do this
        ///          by calling this method with a parameter of 'false'. Just
        ///          remember to manually call the recalcTangents method when
        ///          you are done.
        /// \param[in] _autoCalc If true, tangents are calculated for you
        /// whenever a point changes. If false, you must call reclacTangents to
        /// recalculate them when it best suits.
        public: void SetAutoCalculate(bool _autoCalc);

      /// \brief Recalculates the tangents associated with this spline.
      /// \remarks If you tell the spline not to update on demand by calling
      ///          setAutoCalculate(false) then you must call this after
      ///          completing your updates to the spline points.
      public: void RecalcTangents();

      /// \brief Automatic recalcultation of tangeants when control points are
      /// updated
      protected: bool autoCalc;

      /// \brief the control points
      protected: std::vector<Quaternion> points;

      /// \brief the tangents
      protected: std::vector<Quaternion> tangents;
    };
    /// \}
  }
}

#endif
