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
#ifndef GAZEBO_GUI_BUILDING_BUILDINGEDITORWIDGET_HH_
#define GAZEBO_GUI_BUILDING_BUILDINGEDITORWIDGET_HH_

#include <memory>

#include "gazebo/gui/qt.h"

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    // Forward declare private data.
    class BuildingEditorWidgetPrivate;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class BuildingEditorWidget BuildingEditorWidget.hh
    /// \brief The parent widget of the building editor, level widget and scale
    /// widget.
    class GZ_GUI_VISIBLE BuildingEditorWidget : public QWidget
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

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<BuildingEditorWidgetPrivate> dataPtr;
    };
    /// \}
  }
}

#endif
