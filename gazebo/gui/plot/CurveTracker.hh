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

#ifndef _GAZEBO_GUI_PLOT_CURVE_TRACKER_HH_
#define _GAZEBO_GUI_PLOT_CURVE_TRACKER_HH_

#include <qwt/qwt_plot.h>
#include <qwt/qwt_scale_widget.h>
#include <qwt/qwt_plot_panner.h>
#include <qwt/qwt_plot_layout.h>
#include <qwt/qwt_plot_grid.h>
#include <qwt/qwt_plot_canvas.h>
#include <qwt/qwt_plot_curve.h>
#include <qwt/qwt_plot_marker.h>
#include <qwt/qwt_curve_fitter.h>
#include <qwt/qwt_symbol.h>
#include <qwt/qwt_legend.h>
#include <qwt/qwt_legend_item.h>
#include <qwt/qwt_plot_directpainter.h>
#include <qwt/qwt_plot_magnifier.h>
#include <qwt/qwt_plot_picker.h>
#include <qwt/qwt_picker_machine.h>


namespace gazebo
{
  namespace gui
  {

    /// \brief Track the mouse position to show values
    class CurveTracker: public QwtPlotPicker
    {

      /// \brief constructor
      /// \param[in] _canvas The plot canvas element
      public: CurveTracker(QwtPlotCanvas *_canvas);

      /// \brief Called to get the tracker text to display
      /// \param[in] _pos The mouse position
      /// \return The text to display
      protected: virtual QwtText trackerTextF( const QPointF &_pos ) const;

      /// \brief Called to get the position where the hover text
      /// is to be displayed
      /// \param[in] _font The font
      /// \return The rectangle
      protected:  virtual QRect trackerRect( const QFont &_font ) const;

      /// \brief Returns the curve information at a point
      /// \param[in] _curve The curve instance
      /// \param[in] _pos The mouse position
      /// \return The curve information.
      private: QString curveInfoAt( const QwtPlotCurve *_curve,
                             const QPointF &_pos ) const;

      /// \brief Returns the curve line segment at position x
      /// \param[in] _curve The curve instance
      /// \param[in] _x The x mouse position
      /// \return The line segment
      private: QLineF curveLineAt( const QwtPlotCurve *_curve,
                                   double _x ) const;

      /// \brief Get the highest series sample index.
      /// this is ported from a more recent version of qwt.
      /// \param[in] _series The series data
      /// \param[in] _value The horizontal mouse position
      /// \return The line segment index
      private: int UpperSampleIndex( const QwtSeriesData<QPointF> &_series,
                                     double _value) const;

      /// \brief Draws the (vertical) line on the chart
      /// \param[in] _painter
      private: virtual void drawRubberBand( QPainter *_painter ) const;
    };
  }
}

#endif
