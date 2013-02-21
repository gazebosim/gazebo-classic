/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include "gazebo/math/Vector2d.hh"

#include "gazebo/gui/qwt/qwt_plot.h"
#include "gazebo/gui/qt.h"

class QwtPlotCurve;
class QwtPlotDirectPainter;

namespace gazebo
{
  namespace gui
  {
    /// \brief A plotting widget that handles incremental addition of data.
    class IncrementalPlot : public QwtPlot
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

      /// \brief Add a new point to the plot.
      /// \param[in] _pt Point to add
      public slots: void Add(const QPointF &_pt);

      /// \brief Clear all points from the plot.
      public: void Clear();

      /// \brief The curve to draw.
      private: QwtPlotCurve *curve;

      /// \brief Drawing utility
      private: QwtPlotDirectPainter *directPainter;

      private: std::list<double> history;
    };
  }
}

#endif
