/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#ifndef _BUILDING_EDITOR_WIDGET_HH_
#define _BUILDING_EDITOR_WIDGET_HH_

#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class LevelWidget;
    class ScaleWidget;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class BuildingEditorWidget BuildingEditorWidget.hh
    /// \brief The parent widget of the building editor, level widget and scale
    /// widget.
    class GAZEBO_VISIBLE BuildingEditorWidget : public QWidget
    {
      /// \brief Constructor
      /// \param[in] _parent Parent QWidget.
      public: BuildingEditorWidget(QWidget *_parent = 0);

      /// \brief Destructor
      public: ~BuildingEditorWidget();

      /// \brief Qt event received when the widget is being resized
      /// \param[in] _event Resize event.
      private: void resizeEvent(QResizeEvent *_event);

      /// \brief Custom rotation cursor
      public: static QCursor rotateCursor;

      /// \brief A widget to display and change building levels.
      private: LevelWidget *levelWidget;

      /// \brief A widget to display the scale of the 2D editor view.
      private: ScaleWidget *scaleWidget;

      /// \brief Qt Graphics Scene where graphics items are drawn in
      private: QGraphicsScene *scene;

      /// \brief Minimum width of the Qt graphics scene
      private: int minimumWidth;

      /// \brief Minimum height of the Qt graphics scene
      private: int minimumHeight;
    };
    /// \}
  }
}

#endif
