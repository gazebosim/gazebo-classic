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
#ifndef _GAZEBO_GUI_PLOT_PLOTPALETTE_HH_
#define _GAZEBO_GUI_PLOT_PLOTPALETTE_HH_

#include <memory>

#include "gazebo/gui/ConfigWidget.hh"
#include "gazebo/gui/qt.h"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    // Forward declare private data class
    class PlotPalettePrivate;

    /// \brief A special list widget that allows dragging of items from it to a
    /// plot
    class GZ_GUI_VISIBLE PlotPalette : public QWidget
    {
      Q_OBJECT

      public: PlotPalette(QWidget *_parent);
      public: ~PlotPalette();

      private: void FillTopicsTop();
      private: void FillModelsTop();

      private slots: void OnModelClicked();

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<PlotPalettePrivate> dataPtr;
    };

    // Forward declare private data class
//    class DragableListWidgetPrivate;

    /// \brief A special list widget that allows dragging of items from it to a
    /// plot
    class GZ_GUI_VISIBLE DragableListWidget : public QListWidget
    {
      // Q_OBJECT

      public: DragableListWidget(QWidget *_parent);

      protected: virtual void startDrag(Qt::DropActions _supportedActions);

      protected: virtual Qt::DropActions supportedDropActions();

      /// \internal
      /// \brief Pointer to private data.
//      private: std::unique_ptr<DragableListWidgetPrivate> dataPtr;
    };

    class GZ_GUI_VISIBLE ItemConfigWidget : public ConfigChildWidget
    {
      Q_OBJECT

      public: ItemConfigWidget(const std::string &_text);
      public: ~ItemConfigWidget();
      signals: void Clicked();
      protected: virtual void mouseReleaseEvent(QMouseEvent *_event);
    };
  }
}
#endif
