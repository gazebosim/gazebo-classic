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

#ifndef _GAZEBO_GUI_BUILDINGEDITOR_PRIVATE_HH_
#define _GAZEBO_GUI_BUILDINGEDITOR_PRIVATE_HH_

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class BuildingEditorPalette;
    class BuildingEditorWidget;

    /// \internal
    /// \brief Private data for the BuildingEditor class
    class BuildingEditorPrivate
    {
      /// \brief Contains all the building editor tools.
      public: BuildingEditorPalette *buildingPalette;

      /// \brief Building editor widget for creating a building model
      public: BuildingEditorWidget *buildingEditorWidget;

      /// \brief Our custom menubar
      public: QMenuBar *menuBar;

      /// \brief Action to save model.
      public: QAction *saveAct;

      /// \brief Action to save model as.
      public: QAction *saveAsAct;

      /// \brief Action to start a new model.
      public: QAction *newAct;

      /// \brief Action to exit the editor.
      public: QAction *exitAct;

      /// \brief Save the main window paused state to use when returning.
      public: bool mainWindowPaused;

      /// \brief Label which shows tips when clicked or hovered.
      public: QLabel *tipsLabel;
    };
  }
}

#endif
