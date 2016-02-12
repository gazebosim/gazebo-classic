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
#ifndef _GAZEBO_GUI_PLOT_PLOTCANVAS_HH_
#define _GAZEBO_GUI_PLOT_PLOTCANVAS_HH_

#include <memory>
#include <string>

#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    // Forward declare private data class
    class PlotCanvasPrivate;

    /// \brief Plot canvas
    class GZ_GUI_VISIBLE PlotCanvas : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor.
      /// \param[in] _parent Pointer to the parent widget.
      public: PlotCanvas(QWidget *_parent);

      /// \brief Destructor.
      public: virtual ~PlotCanvas();

      /// \brief Update plots.
      public: void Update();

      /// \brief Used to filter scroll wheel events.
      /// \param[in] _o Object that receives the event.
      /// \param[in] _event Pointer to the event.
      public: virtual bool eventFilter(QObject *_o, QEvent *_e);

      /// \brief Qt signal to request self-deletion.
      Q_SIGNALS: void CanvasDeleted();

      /// \brief Qt Callback to clear all variable and plots on canvas.
      private slots: void OnClearCanvas();

      /// \brief Qt Callback to delete entire canvas.
      private slots: void OnDeleteCanvas();

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<PlotCanvasPrivate> dataPtr;
    };
  }
}
#endif
