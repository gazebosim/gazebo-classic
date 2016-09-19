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
#ifndef GAZEBO_GUI_PLOT_INCREMENTALPLOT_HH_
#define GAZEBO_GUI_PLOT_INCREMENTALPLOT_HH_

#include <memory>
#include <string>
#include <vector>

#include <ignition/math/Vector2.hh>

#include "gazebo/gui/qt.h"
#include "gazebo/gui/plot/qwt_gazebo.h"
#include "gazebo/gui/plot/PlottingTypes.hh"
#include "gazebo/util/system.hh"

class QwtPlotCurve;

namespace gazebo
{
  namespace common
  {
    class Time;
  }

  namespace gui
  {
    // Forward declare private data class
    class IncrementalPlotPrivate;

    /// \brief A plotting widget that handles incremental addition of data.
    class GZ_GUI_VISIBLE IncrementalPlot : public QwtPlot
    {
      Q_OBJECT

      /// \brief Axis enum
      public: enum PlotAxis
      {
        /// \brief bottom x axis
        X_BOTTOM_AXIS = 0,

        /// \brief top x axis
        X_TOP_AXIS = 1,

        /// \brief left y axis
        Y_LEFT_AXIS = 2,

        /// \brief right y axis
        Y_RIGHT_AXIS = 3
      };

      /// \brief Constructor
      /// \param[in] _parent Pointer to a parent widget
      public: IncrementalPlot(QWidget *_parent = nullptr);

      /// \brief Destructor
      public: virtual ~IncrementalPlot();

     /// \brief Add a named curve.
      /// \param[in] _label Name of the curve.
      /// \return A pointer to the new curve.
      public: PlotCurveWeakPtr AddCurve(const std::string &_label);

      /// \brief Add a new point to a curve.
      /// \param[in] _id Unique id of the curve
      /// \param[in] _pt Point to add.
      public: void AddPoint(const unsigned int _id,
          const ignition::math::Vector2d &_pt);

      /// \brief Add new points to a curve.
      /// \param[in] _id Unique id of the curve
      /// \param[in] _pt Points to add.
      public: void AddPoints(const unsigned int _id,
          const std::vector<ignition::math::Vector2d> &_pts);

      /// \brief Clear all points from the plot.
      public: void Clear();

      /// \brief Find a plot curve by name
      /// \param[in] _label Name of the curve to look for.
      /// \return Plot curve if found, nullptr otherwise.
      public: PlotCurveWeakPtr Curve(const std::string &_label) const;

      /// \brief Find a plot curve by id
      /// \param[in] _id Unique id of the plot curve.
      /// \return Plot curve if found, nullptr otherwise.
      public: PlotCurveWeakPtr Curve(const unsigned int _id) const;

      /// \brief Update all the curves in the plot
      public: void Update();

      /// \brief Remove a curve by id
      /// \param[in] _id Unique id of the curve.
      public: void RemoveCurve(const unsigned int _id);

      /// \brief Set the period over which to plot.
      /// \param[in] _time Period duration in seconds.
      public: void SetPeriod(const common::Time &_time);

      /// \brief Attach a curve to this plot.
      /// \param[in] _plotCurve The curve to attach to the plot.
      public: void AttachCurve(PlotCurveWeakPtr _curve);

      /// \brief Detach a curve from this plot.
      /// \param[in] _id Unique id of the plot curve to detach.
      /// \return Pointer to the plot curve
      public: PlotCurvePtr DetachCurve(const unsigned int _id);

      /// \brief Set a new label for the given curve.
      /// \param[in] _id Unique id of the plot curve
      /// \param[in] _label New label to set the plot curve to.
      public: void SetCurveLabel(const unsigned int _id,
        const std::string &_label);

      /// \brief Set whether to show the axis label.
      /// \param[in] _axis Plot axis: X_BOTTOM_AXIS or Y_LEFT_AXIS.
      /// \param[in] _show True to show the label, false to hide it.
      public: void ShowAxisLabel(const PlotAxis _axis, const bool _show);

      /// \brief Set whether to show the grid lines.
      /// \param[in] _show True to show grid lines.
      public: void ShowGrid(const bool _show);

      /// \brief Get whether the grid lines are shown.
      /// \return True if the grid lines are visible.
      public: bool IsShowGrid() const;

      /// \brief Set whether to show the hover line.
      /// \param[in] _show True to show hover line.
      public: void ShowHoverLine(const bool _show);

      /// \brief Get whether the hover line is shown.
      /// \return True if the hover line is visible.
      public: bool IsShowHoverLine() const;

      /// \brief Get all curves in this plot
      /// \return A list of curves in this plot.
      public: std::vector<PlotCurveWeakPtr> Curves() const;

      /// \brief Give QT a size hint.
      /// \return Default size of the plot.
      public: virtual QSize sizeHint() const;

      /// \brief Used to accept drag enter events.
      /// \param[in] _evt The drag event.
      protected: void dragEnterEvent(QDragEnterEvent *_evt);

      /// \brief Used to accept drop events.
      /// \param[in] _evt The drop event.
      protected: void dropEvent(QDropEvent *_evt);

      /// \brief Qt signal emitted when a variable pill is added
      /// \param[in] _name Name of variable pill added.
      Q_SIGNALS: void VariableAdded(const std::string &_name);

      /// \internal
      /// \brief Private data pointer
      private: std::unique_ptr<IncrementalPlotPrivate> dataPtr;
    };
  }
}
#endif
