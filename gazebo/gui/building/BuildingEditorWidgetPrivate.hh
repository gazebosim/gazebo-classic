/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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

#ifndef _GAZEBO_GUI_BUILDING_EDITOR_WIDGET_PRIVATE_HH_
#define _GAZEBO_GUI_BUILDING_EDITOR_WIDGET_PRIVATE_HH_

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    // Forward declare pointers.
    class LevelWidget;
    class ScaleWidget;

    /// \internal
    /// \brief Private data for the BuildingEditorWidget class
    class BuildingEditorWidgetPrivate
    {
      /// \brief A widget to display and change building levels.
      public: LevelWidget *levelWidget;

      /// \brief A widget to display the scale of the 2D editor view.
      public: ScaleWidget *scaleWidget;

      /// \brief Qt Graphics Scene where graphics items are drawn in
      public: QGraphicsScene *scene;

      /// \brief Minimum width of the Qt graphics scene
      public: int minimumWidth;

      /// \brief Minimum height of the Qt graphics scene
      public: int minimumHeight;
    };
  }
}

#endif
