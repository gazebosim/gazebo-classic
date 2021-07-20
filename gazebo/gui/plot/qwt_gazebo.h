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

#ifndef GAZEBO_GUI_PLOT_QWT_GAZEBO_H_
#define GAZEBO_GUI_PLOT_QWT_GAZEBO_H_

#pragma GCC system_header

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wfloat-equal"

#if defined __has_include
  #if __has_include (<qwt.h>)
    #include <qwt_curve_fitter.h>
    #include <qwt_global.h>
    #include <qwt_legend.h>
    #include <qwt_painter.h>
    #include <qwt_picker_machine.h>
    #include <qwt_plot.h>
    #include <qwt_plot_canvas.h>
    #include <qwt_plot_curve.h>
    #include <qwt_plot_directpainter.h>
    #include <qwt_plot_grid.h>
    #include <qwt_plot_layout.h>
    #include <qwt_plot_magnifier.h>
    #include <qwt_plot_marker.h>
    #include <qwt_plot_panner.h>
    #include <qwt_plot_zoomer.h>
    #include <qwt_scale_engine.h>
    #include <qwt_scale_map.h>
    #include <qwt_scale_widget.h>
    #include <qwt_symbol.h>
    #include <qwt_plot_renderer.h>
    #define GAZEBO_GUI_QWT_IS_INCLUDED
  #endif
#endif

#ifndef GAZEBO_GUI_QWT_IS_INCLUDED
  #include <qwt/qwt_curve_fitter.h>
  #include <qwt/qwt_global.h>
  #include <qwt/qwt_legend.h>
  #include <qwt/qwt_painter.h>
  #include <qwt/qwt_picker_machine.h>
  #include <qwt/qwt_plot.h>
  #include <qwt/qwt_plot_canvas.h>
  #include <qwt/qwt_plot_curve.h>
  #include <qwt/qwt_plot_directpainter.h>
  #include <qwt/qwt_plot_grid.h>
  #include <qwt/qwt_plot_layout.h>
  #include <qwt/qwt_plot_magnifier.h>
  #include <qwt/qwt_plot_marker.h>
  #include <qwt/qwt_plot_panner.h>
  #include <qwt/qwt_plot_zoomer.h>
  #include <qwt/qwt_scale_engine.h>
  #include <qwt/qwt_scale_map.h>
  #include <qwt/qwt_scale_widget.h>
  #include <qwt/qwt_symbol.h>
  #include <qwt/qwt_plot_renderer.h>
  #define GAZEBO_GUI_QWT_IS_INCLUDED
#endif

#pragma clang diagnostic pop

#endif
