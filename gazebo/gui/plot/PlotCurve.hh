/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_GUI_PLOT_PLOTCURVE_HH_
#define GAZEBO_GUI_PLOT_PLOTCURVE_HH_

#include <memory>
#include <string>
#include <vector>

#include <ignition/math/Vector2.hh>

#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

class QwtPlotCurve;

namespace gazebo
{
  namespace gui
  {
    // Forward declare private data class
    class PlotCurvePrivate;

    class IncrementalPlot;

    /// \brief Plot Curve data.
    class GZ_GUI_VISIBLE PlotCurve
    {
      /// \brief Constructor.
      /// \param[in] _label Plot label.
      public: PlotCurve(const std::string &_label);

      /// \brief Destructor.
      public: ~PlotCurve();

      /// \brief Add a point to the curve
      /// \param[in] _pt Point to add.
      public: void AddPoint(const ignition::math::Vector2d &_pt);

      /// \brief Add points to the curve
      /// \param[in] _pts Points to add.
      public: void AddPoints(const std::vector<ignition::math::Vector2d> &_pt);

      /// \brief Clear all data from the curve.
      public: void Clear();

      /// \brief Attach the curve to a plot.
      /// \param[in] _plot Plot to attach to.
      public: void Attach(IncrementalPlot *_plot);

      /// \brief Detach the curve from the plot.
      public: void Detach();

      /// \brief Set the curve label.
      /// \param[in] _label Label to set the curve to
      public: void SetLabel(const std::string &_label);

      /// \brief Get the curve label.
      /// \return Curve label
      public: std::string Label() const;

      /// \brief Set the curve id.
      /// \param[in] _id Id to set the curve to.
      public: void SetId(const unsigned int _id);

      /// \brief Get the curve id.
      /// \return Curve id
      public: unsigned int Id() const;

      /// \brief Set whether the curve should be active.
      /// \param[in] _active True to make the curve active and
      /// accept new points.
      public: void SetActive(const bool _active);

      /// \brief Get whether the curve is active.
      /// \return Active state of the plot curve
      public: bool Active() const;

      /// \brief Set the age of the curve.
      /// \param[in] _age Age of the curve. Currently used to keep track of
      /// how many restarts it has been through.
      public: void SetAge(const unsigned int _age);

      /// \brief Get the age of the curve.
      /// \return Curve age
      public: unsigned int Age() const;

      /// \brief Get the number of data points in the curve
      /// \return Number of data points.
      public: unsigned int Size() const;

      /// \brief Get the min x and y values of this curve
      /// \return Point with min values
      public: ignition::math::Vector2d Min();

      /// \brief Get the max x and y values of this curve
      /// \return Point with max values
      public: ignition::math::Vector2d Max();

      /// \brief Get a point in the curve
      /// \param[in] _index Index of the point in the curve.
      /// \return 2d point at the specified index. A Vector2d of nans is
      /// returned if the index is out of bounds.
      public: ignition::math::Vector2d Point(const unsigned int _index) const;

      /// \brief Return all the sample points in the curve.
      /// \return Curve sample points
      public: std::vector<ignition::math::Vector2d> Points() const;

      /// \internal
      /// \brief Get the internal QwtPlotCurve object.
      /// \return QwtPlotCurve object.
      public: QwtPlotCurve *Curve();

      /// \internal
      /// \brief Private data pointer
      private: std::unique_ptr<PlotCurvePrivate> dataPtr;
    };
  }
}
#endif
