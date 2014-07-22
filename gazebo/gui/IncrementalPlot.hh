/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#ifndef _INCREMENTAL_PLOT_HH_
#define _INCREMENTAL_PLOT_HH_

#include <map>
#include <list>

#include <qwt/qwt_plot_magnifier.h>
#include <qwt/qwt_plot.h>

#include "gazebo/math/Vector2d.hh"

#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

class QwtPlotCurve;
class QwtPlotDirectPainter;

namespace gazebo
{
  namespace gui
  {
    /// \brief A plotting widget that handles incremental addition of data.
    class GAZEBO_VISIBLE IncrementalPlot : public QwtPlot
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Pointer to a parent widget
      public: IncrementalPlot(QWidget *_parent = NULL);

      /// \brief Destructor
      public: virtual ~IncrementalPlot();

      /// \brief Give QT a size hint.
      /// \return Default size of the plot.
      public: virtual QSize sizeHint() const;

      /// \brief Add a new point to a curve.
      /// \param[in] _label Name of the curve to add a point to. A curve
      /// will be added if it doesn't exist.
      /// \param[in] _pt Point to add.
      public slots: void Add(const QString &_label, const QPointF &_pt);

      /// \brief Add new points to a curve.
      /// \param[in] _label Name of the curve to add a point to. A curve
      /// will be added if it doesn't exist.
      /// \param[in] _pt Points to add.
      public slots: void Add(const QString &_label,
                             const std::list<QPointF> &_pts);

      /// \brief Add a vertical line to the plot.
      /// \param[in] _label Label for the line.
      /// \param[in] _x X position for the vertical line.
      public: void AddVLine(const QString &_label, double _x);

      /// \brief Clear a single curve from the plot.
      /// \param[in] _label Name of the curve to remove.
      public: void Clear(const QString &_label);

      /// \brief Clear all points from the plot.
      public: void Clear();

      /// \brief Return true if the plot has the labled curve.
      /// \param[in] _label Name of the curve to check for.
      /// \return True if _label is currently plotted.
      public: bool HasCurve(const QString &_label);

      /// \brief Update all the curves in the plot
      public: void Update();

      /// \brief Add a named curve.
      /// \param[in] _label Name of the curve.
      /// \return A pointer to the new curve.
      public: QwtPlotCurve *AddCurve(const QString &_label);

      /// \brief Set the period over which to plot.
      /// \param[in] _seconds Period duration in seconds.
      public: void SetPeriod(double _seconds);

      /// \brief Used to accept drag enter events.
      /// \param[in] _evt The drag event.
      protected: void dragEnterEvent(QDragEnterEvent *_evt);

      /// \brief Used to accept drop events.
      /// \param[in] _evt The drop event.
      protected: void dropEvent(QDropEvent *_evt);

      /// \brief Used to handle Qt mouse wheel events.
      /// \param[in] _event Mouse wheel event
      protected: void wheelEvent(QWheelEvent *_event);

      /// \brief Adjust a curve to fit new data.
      /// \param[in] _curve Curve to adjust
      private: void AdjustCurve(QwtPlotCurve *_curve);

      /// \def DiagnosticTimerPtr
      /// \brief A map of strings to qwt plot curves.
      private: typedef std::map<QString, QwtPlotCurve *> CurveMap;

      /// \brief The curve to draw.
      private: CurveMap curves;

      /// \brief Drawing utility
      private: QwtPlotDirectPainter *directPainter;

      /// \brief Pointer to the plot maginfier
      private: QwtPlotMagnifier *magnifier;

      /// \brief Period duration in seconds.
      private: double period;
    };
  }
}
#endif
