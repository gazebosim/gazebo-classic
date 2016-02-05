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

#ifndef _GAZEBO_GUI_INCREMENTAL_PLOT_HH_
#define _GAZEBO_GUI_INCREMENTAL_PLOT_HH_

#include <memory>
#include <string>
#include <vector>

#include <qwt/qwt_plot.h>

#include <ignition/math/Vector2.hh>

#include "gazebo/gui/qt.h"
#include "gazebo/gui/plot/PlottingTypes.hh"
#include "gazebo/util/system.hh"

class QwtPlotCurve;

namespace gazebo
{
  namespace gui
  {
    // Forward declare private data class
    struct IncrementalPlotPrivate;

    /// \brief A plotting widget that handles incremental addition of data.
    class GZ_GUI_VISIBLE IncrementalPlot : public QwtPlot
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Pointer to a parent widget
      public: IncrementalPlot(QWidget *_parent = NULL);

      /// \brief Destructor
      public: virtual ~IncrementalPlot();

      /// \brief Add a new point to a curve.
      /// \param[in] _label Name of the curve to add a point to. A curve
      /// will be added if it doesn't exist.
      /// \param[in] _pt Point to add.
      public:  void Add(const std::string &_label,
          const ignition::math::Vector2d &_pt);

      /// \brief Add new points to a curve.
      /// \param[in] _label Name of the curve to add a point to. A curve
      /// will be added if it doesn't exist.
      /// \param[in] _pt Points to add.
      public: void Add(const std::string &_label,
          const std::vector<ignition::math::Vector2d> &_pts);

      /// \brief Add a vertical line to the plot.
      /// \param[in] _label Label for the line.
      /// \param[in] _x X position for the vertical line.
      public: void AddVLine(const std::string &_label, const double _x);

      /// \brief Clear a single curve from the plot.
      /// \param[in] _label Name of the curve to remove.
      public: void Clear(const std::string &_label);

      /// \brief Clear all points from the plot.
      public: void Clear();

      /// \brief Return true if the plot has the labeled curve.
      /// \param[in] _label Name of the curve to check for.
      /// \return True if _label is currently plotted.
      public: bool HasCurve(const std::string &_label);

      /// \brief Find a plot curve by name
      /// \param[in] _label Name of the curve to look for.
      /// \return Plot curve if found, NULL otherwise.
      public: PlotCurveWeakPtr Curve(const std::string &_label) const;

      /// \brief Find a plot curve by id
      /// \param[in] _id Unique id of the plot curve.
      /// \return Plot curve if found, NULL otherwise.
      public: PlotCurveWeakPtr Curve(const unsigned int _id) const;

      /// \brief Update all the curves in the plot
      public: void Update();

      /// \brief Add a named curve.
      /// \param[in] _label Name of the curve.
      /// \return A pointer to the new curve.
      public: PlotCurveWeakPtr AddCurve(const std::string &_label);

      /// \brief Remove a curve by id
      /// \param[in] _id Unique id of the curve.
      public: void RemoveCurve(const unsigned int _id);

      /// \brief Set the period over which to plot.
      /// \param[in] _seconds Period duration in seconds.
      public: void SetPeriod(const unsigned int _seconds);

      /// \brief Attach a curve to this plot.
      /// \param[in] _plotCurve The curve to attach to the plot.
      public: void AttachCurve(PlotCurveWeakPtr _curve);

      /// \brief Dettach a curve from this plot.
      /// \param[in] _id Unique id of the plot curve to detach.
      /// \return Pointer to the plot curve
      public: PlotCurvePtr DetachCurve(const unsigned int _id);

      /// \brief Set a new label for the given curve.
      /// \param[in] _id Unique id of the plot curve
      /// \param[in] _label New label to set the plot curve to.
      public: void SetCurveLabel(const unsigned int _id,
        const std::string &_label);

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

      /// \brief Adjust a curve to fit new data.
      /// \param[in] _curve Curve to adjust
      private: void AdjustCurve(PlotCurvePtr _curve);

      /// \internal
      /// \brief Private data pointer
      private: std::unique_ptr<IncrementalPlotPrivate> dataPtr;
    };
  }
}
#endif
