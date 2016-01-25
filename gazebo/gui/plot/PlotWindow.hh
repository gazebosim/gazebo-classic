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
#ifndef _GAZEBO_GUI_PLOTWINDOW_HH_
#define _GAZEBO_GUI_PLOTWINDOW_HH_

#include <memory>

#include "gazebo/gui/qt.h"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    // Forward declare private data class
    class PlotWindowPrivate;

    class PlotCanvas;

    /// \brief Plot diagnostic information
    class GZ_GUI_VISIBLE PlotWindow : public QDialog
    {
      Q_OBJECT

      /// \brief Constructor.
      /// \param[in] _parent Pointer to the parent widget.
      public: PlotWindow(QWidget *_parent = NULL);

      /// \brief Destructor.
      public: virtual ~PlotWindow();

      /// \brief Used to filter scroll wheel events.
      /// \param[in] _o Object that receives the event.
      /// \param[in] _event Pointer to the event.
      public: virtual bool eventFilter(QObject *_o, QEvent *_e);

      /// \brief QT close event, used to stop PlotWindow.
      /// \param[in] _evt The close event.
      protected: virtual void closeEvent(QCloseEvent *_evt);

      /// \brief Called when a diagnostic message is received.
      /// \param[in] _msg Diagnostic message.
      // private: void OnMsg(ConstPlotWindowPtr &_msg);

      /// \brief Add a new plot canvas.
      public: PlotCanvas *AddCanvas();

      /// \brief Remove a plot canvas
      public: void RemoveCanvas(PlotCanvas *canvas);

      /// \brief QT callback to continue plotting.
      private slots: void OnPlay();

      /// \brief QT callback for when plotting is to be paused.
      private slots: void OnPause();

      /// \brief Qt Callback when a new plot canvas should be added.
      private slots: void OnAddCanvas();

      /// \brief Qt Callback when a plot canvas should be removed.
      private slots: void OnRemoveCanvas();

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<PlotWindowPrivate> dataPtr;
    };
  }
}
#endif
