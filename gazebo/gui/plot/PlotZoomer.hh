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

#ifndef _GAZEBO_GUI_PLOT_PLOTZOOMER_HH_
#define _GAZEBO_GUI_PLOT_PLOTZOOMER_HH_

#include <memory>

#include "gazebo/gui/plot/qwt_gazebo.h"

namespace gazebo
{
  namespace gui
  {
    class PlotZoomerPrivate;

    /// \brief Box zoom with mouse hover tracking
    class PlotZoomer: public QwtPlotZoomer
    {
      /// \brief Constructor
      /// \param[in] _canvas Canvas the zoomer will be attached to.
#if (QWT_VERSION < ((6 << 16) | (1 << 8) | 0))
      public: PlotZoomer(QwtPlotCanvas *_canvas);
#else
      public: PlotZoomer(QWidget *_canvas);
#endif

      /// \brief Update the hover text, also known as the tracker text.
      public: void Update();

      /// \brief Overriden to update the hover line in addition to the default
      /// widgets (rubberband and tracker text).
      public: void updateDisplay();

      /// \brief Overriden to provide customized hover text
      /// \param[in] _pos Mouse position
      /// \return Text to display
      protected: virtual QwtText trackerTextF(const QPointF &_pos) const;

      /// \brief Get curve information at a point
      /// \param[in] _curve Plot curve instance
      /// \param[in] _pos Mouse position
      /// \return Curve information.
      private: QString CurveInfoAt(const QwtPlotCurve *_curve,
                                   const QPointF &_pos) const;

      /// \brief Get the curve line segment at position x
      /// \param[in] _curve Plot curve instance
      /// \param[in] _x X mouse position
      /// \return Line segment
      private: QLineF CurveLineAt(const QwtPlotCurve *_curve,
                                  const double _x) const;

      /// \brief Get the index of sample that is the upper bound of value.
      /// This is ported from qwt-6.1.
      /// \param[in] _series The plot series data
      /// \param[in] _value Value to search
      /// \return Sample index.
      private: int UpperSampleIndex(const QwtSeriesData<QPointF> &_series,
                                    const double _value) const;

      /// \internal
      /// \brief Private data pointer
      private: std::unique_ptr<PlotZoomerPrivate> dataPtr;
    };
  }
}

#endif
