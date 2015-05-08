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

#ifndef _BUILDING_EDITOR_PALETTE_PRIVATE_HH_
#define _BUILDING_EDITOR_PALETTE_PRIVATE_HH_

#include <vector>
#include <string>
#include <map>
#include "gazebo/common/Events.hh"
#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    /// \internal
    /// \brief Private data for BuildingEditorPalette
    class BuildingEditorPalettePrivate
    {
      /// \brief Custom color dialog.
      public: QColorDialog *customColorDialog;

      /// \brief Custom color button.
      public: QPushButton *customColorButton;

      /// \brief Link each button ID to a draw mode.
      public: std::map<std::string, int> brushIdToModeMap;

      /// \brief Name of the last default texture mode.
      public: std::string lastDefaultTexture;

      /// \brief Name of the last default color mode.
      public: std::string lastDefaultColor;

      /// \brief Default name of the building model.
      public: std::string buildingDefaultName;

      /// \brief Edit the name of the building model.
      public: QLineEdit *modelNameEdit;

      /// \brief All the brushes (wall, door, window, stair, etc).
      public: QButtonGroup *brushes;

      /// \brief A list of gui editor events connected to this palette.
      public: std::vector<event::ConnectionPtr> connections;

      /// \brief The current draw mode, empty for none.
      public: std::string currentMode;

      /// \brief List of default colors to be picked.
      public: std::vector<QColor> colorList;

      /// \brief List of default textures to be picked.
      public: std::vector<QString> textureList;
    };
  }
}

#endif
