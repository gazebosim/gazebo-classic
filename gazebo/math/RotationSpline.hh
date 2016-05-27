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
#ifndef _ROTATIONSPLINE_HH_
#define _ROTATIONSPLINE_HH_

#include <vector>

#include "gazebo/math/Quaternion.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace math
  {
    /// \addtogroup gazebo_math
    /// \{

    /// \class RotationSpline RotationSpline.hh math/gzmath.hh
    /// \brief Spline for rotations
    class GZ_MATH_VISIBLE  RotationSpline
    {
        /// \brief Constructor. Sets the autoCalc to true
        /// \deprecated See ignition::math::RotationSpline
        public: RotationSpline() GAZEBO_DEPRECATED(8.0);

        /// \brief Destructor. Nothing is done
        /// \deprecated See ignition::math::RotationSpline
        public: ~RotationSpline() GAZEBO_DEPRECATED(8.0);

        /// \brief Adds a control point to the end of the spline.
        /// \param[in] _p control point
        /// \deprecated See ignition::math::RotationSpline
        public: void AddPoint(const Quaternion &_p) GAZEBO_DEPRECATED(8.0);

        /// \brief Gets the detail of one of the control points of the spline.
        /// \param[in] _index the index of the control point.
        /// \remarks This point must already exist in the spline.
        /// \return a quaternion (out of bound index result in assertion)
        /// \deprecated See ignition::math::RotationSpline
        public: const Quaternion &GetPoint(unsigned int _index) const
            GAZEBO_DEPRECATED(8.0);

        /// \brief Gets the number of control points in the spline.
        /// \return the count
        /// \deprecated See ignition::math::RotationSpline
        public: unsigned int GetNumPoints() const GAZEBO_DEPRECATED(8.0);

        /// \brief Clears all the points in the spline.
        /// \deprecated See ignition::math::RotationSpline
        public: void Clear() GAZEBO_DEPRECATED(8.0);

        /// \brief Updates a single point in the spline.
        /// \remarks This point must already exist in the spline.
        /// \param[in] _index index
        /// \param[in] _value the new control point value
        /// \deprecated See ignition::math::RotationSpline
        public: void UpdatePoint(unsigned int _index, const Quaternion &_value)
            GAZEBO_DEPRECATED(8.0);

        /// \brief Returns an interpolated point based on a parametric
        ///        value over the whole series.
        /// \remarks Given a t value between 0 and 1 representing the
        ///          parametric distance along the whole length of the spline,
        ///          this method returns an interpolated point.
        /// \param[in] _t Parametric value.
        /// \param[in] _useShortestPath Defines if rotation should take the
        ///        shortest possible path
        /// \return the rotation
        /// \deprecated See ignition::math::RotationSpline
        public: Quaternion Interpolate(double _t, bool _useShortestPath = true)
            GAZEBO_DEPRECATED(8.0);

        /// \brief Interpolates a single segment of the spline
        ///        given a parametric value.
        /// \param[in] _fromIndex The point index to treat as t = 0.
        ///        _fromIndex + 1 is deemed to be t = 1
        /// \param[in] _t Parametric value
        /// \param[in] _useShortestPath Defines if rotation should take the
        ///         shortest possible path
        /// \return the rotation
        /// \deprecated See ignition::math::RotationSpline
        public: Quaternion Interpolate(unsigned int _fromIndex, double _t,
            bool _useShortestPath = true) GAZEBO_DEPRECATED(8.0);

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
        /// \deprecated See ignition::math::RotationSpline
        public: void SetAutoCalculate(bool _autoCalc) GAZEBO_DEPRECATED(8.0);

      /// \brief Recalculates the tangents associated with this spline.
      /// \remarks If you tell the spline not to update on demand by calling
      ///          setAutoCalculate(false) then you must call this after
      ///          completing your updates to the spline points.
      /// \deprecated See ignition::math::RotationSpline
      public: void RecalcTangents() GAZEBO_DEPRECATED(8.0);

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
