/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
// Note: Originally cribbed from Ogre3d. Modified to implement Cardinal
// spline and catmull-rom spline
#ifndef _SPLINE_HH_
#define _SPLINE_HH_

#include <vector>

#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Matrix4.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace math
  {
    /// \addtogroup gazebo_math
    /// \{

    /// \class Spline Spline.hh math/gzmath.hh
    /// \brief Splines
    class GZ_MATH_VISIBLE Spline
    {
      /// \brief constructor
      /// \deprecated See ignition::math::Spline
      public: Spline() GAZEBO_DEPRECATED(8.0);

      /// \brief destructor
      /// \deprecated See ignition::math::Spline
      public: ~Spline() GAZEBO_DEPRECATED(8.0);

      /// \brief Set the tension parameter. A value of 0 = Catmull-Rom
      /// spline.
      /// \param[in] _t Tension value between 0.0 and 1.0
      /// \deprecated See ignition::math::Spline
      public: void SetTension(double _t) GAZEBO_DEPRECATED(8.0);

      /// \brief Get the tension value
      /// \return The value of the tension, which is between 0.0 and 1.0
      /// \deprecated See ignition::math::Spline
      public: double GetTension() const GAZEBO_DEPRECATED(8.0);

      /// \brief  Adds a control point to the end of the spline.
      /// \param[in] _pt point to add
      /// \deprecated See ignition::math::Spline
      public: void AddPoint(const Vector3 &_pt) GAZEBO_DEPRECATED(8.0);

      /// \brief Gets the detail of one of the control points of the spline.
      /// \param[in] _index the control point index
      /// \return the control point, or [0,0,0] and a message on the error
      /// stream
      /// \deprecated See ignition::math::Spline
      public: Vector3 GetPoint(unsigned int _index) const
          GAZEBO_DEPRECATED(8.0);

      /// \brief  Gets the number of control points in the spline.
      /// \return the count
      /// \deprecated See ignition::math::Spline
      public: unsigned int GetPointCount() const GAZEBO_DEPRECATED(8.0);

      /// \brief Get the tangent value for a point
      /// \param[in] _index the control point index
      /// \deprecated See ignition::math::Spline
      public: Vector3 GetTangent(unsigned int _index) const
           GAZEBO_DEPRECATED(8.0);

      /// \brief  Clears all the points in the spline.
      /// \deprecated See ignition::math::Spline
      public: void Clear() GAZEBO_DEPRECATED(8.0);

      /// \brief Updates a single point in the spline.
      /// \remarks an error to the error stream is printed when the index is
      /// out of bounds
      /// \param[in] _index the control point index
      /// \param[in] _value the new position
      /// \deprecated See ignition::math::Spline
      public: void UpdatePoint(unsigned int _index, const Vector3 &_value)
          GAZEBO_DEPRECATED(8.0);

      /// \brief Returns an interpolated point based on a parametric value
      ///        over the whole series.
      /// \param[in] _t parameter (range 0 to 1)
      /// \deprecated See ignition::math::Spline
      public: Vector3 Interpolate(double _t) const GAZEBO_DEPRECATED(8.0);

      /// \brief Interpolates a single segment of the spline given a
      ///        parametric value.
      /// \param[in] _fromIndex The point index to treat as t = 0.
      ///        fromIndex + 1 is deemed to be t = 1
      /// \param[in] _t Parametric value
      /// \deprecated See ignition::math::Spline
      public: Vector3 Interpolate(unsigned int _fromIndex, double _t) const
          GAZEBO_DEPRECATED(8.0);

      /// \brief Tells the spline whether it should automatically
      ///        calculate tangents on demand as points are added.
      /// \remarks The spline calculates tangents at each point
      ///          automatically based on the input points. Normally it
      ///          does this every time a point changes. However, if you
      ///          have a lot of points to add in one go, you probably
      ///          don't want to incur this overhead and would prefer to
      ///          defer the calculation until you are finished setting all
      ///          the points. You can do this by calling this method with a
      ///          parameter of 'false'. Just remember to manually call the
      ///          recalcTangents method when you are done.
      /// \param[in] _autoCalc If true, tangents are calculated for you whenever
      ///        a point changes. If false, you must call reclacTangents to
      ///        recalculate them when it best suits.
      /// \deprecated See ignition::math::Spline
      public: void SetAutoCalculate(bool _autoCalc) GAZEBO_DEPRECATED(8.0);

      /// \brief Recalculates the tangents associated with this spline.
      /// \remarks If you tell the spline not to update on demand by
      ///          calling setAutoCalculate(false) then you must call this
      ///          after completing your updates to the spline points.
      /// \deprecated See ignition::math::Spline
      public: void RecalcTangents() GAZEBO_DEPRECATED(8.0);

      /// \brief when true, the tangents are recalculated when the control
      /// point change
      protected: bool autoCalc;

      /// \brief control points
      protected: std::vector<Vector3> points;

      /// \brief tangents
      protected: std::vector<Vector3> tangents;

      /// Matrix of coefficients
      protected: Matrix4 coeffs;

      /// Tension of 0 = Catmull-Rom spline, otherwise a Cardinal spline
      protected: double tension;
    };
    /// \}
  }
}
#endif
