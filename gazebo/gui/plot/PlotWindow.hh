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
#ifndef _GAZEBO_GUI_PLOT_PLOTWINDOW_HH_
#define _GAZEBO_GUI_PLOT_PLOTWINDOW_HH_

#include <memory>

#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    // Forward declare private data class
    class PlotWindowPrivate;

    class PlotCanvas;

    /// \brief Plot window
    class GZ_GUI_VISIBLE PlotWindow : public QDialog
    {
      Q_OBJECT

      /// \brief Constructor.
      /// \param[in] _parent Pointer to the parent widget.
      public: PlotWindow(QWidget *_parent = NULL);

      /// \brief Destructor.
      public: virtual ~PlotWindow();

      /// \brief Add a new canvas.
      public: PlotCanvas *AddCanvas();

      /// \brief Remove a plot canvas
      /// \param[in] _canvas Canvas to remove
      public: void RemoveCanvas(PlotCanvas *_canvas);

      /// \brief Get the number of canvases in this plot window.
      /// \return Number of canvases
      public: unsigned int CanvasCount() const;

      /// \brief Clear and remove all canvases
      public: void Clear();

      /// \brief Restart plotting. A new plot curve will be created for each
      /// variable in the plot. Existing plot curves will no longer be updated.
      public: void Restart();

      /// \brief Export all canvas plots.
      public: void Export();

      /// \brief Update all canvases
      public slots: void Update();

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
