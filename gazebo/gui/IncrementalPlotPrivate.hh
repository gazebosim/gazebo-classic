/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef _GAZEBO_GUI_INCREMENTALPLOT_PRIVATE_HH_
#define _GAZEBO_GUI_INCREMENTALPLOT_PRIVATE_HH_

#include <map>

#include <qwt/qwt_plot_magnifier.h>
#include <qwt/qwt_plot.h>

namespace gazebo
{
  namespace gui
  {
    /// \class IncrementalPlotPrivate IncrementalPlotPrivate.hh
    /// \brief Private data for the IncrementalPlot class.
    class IncrementalPlotPrivate
    {
      /// \def DiagnosticTimerPtr
      /// \brief A map of strings to qwt plot curves.
      public: typedef std::map<QString, QwtPlotCurve *> CurveMap;

      /// \brief The curve to draw.
      public: CurveMap curves;

      /// \brief Drawing utility
      public: QwtPlotDirectPainter *directPainter;

      /// \brief Pointer to the plot maginfier
      public: QwtPlotMagnifier *magnifier;
    };
  }
}
#endif
