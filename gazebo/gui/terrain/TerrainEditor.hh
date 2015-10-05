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
#ifndef _TERRAIN_EDITOR_HH_
#define _TERRAIN_EDITOR_HH_

#include "gazebo/gui/qt.h"
#include "gazebo/gui/Editor.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class TerrainEditorPalette;

    /// \class TerrainEditor TerrainEditor.hh gui/gui.hh
    /// \brief Interface to the terrain editor.
    class GZ_GUI_TERRAIN_VISIBLE TerrainEditor : public Editor
    {
      Q_OBJECT

      /// \brief Constuctor.
      /// \param[in] _mainWindow Pointer to the mainwindow.
      public: TerrainEditor(MainWindow *_mainWindow);

      /// \brief Destuctor.
      public: virtual ~TerrainEditor();

      /// \brief QT callback when entering building edit mode
      /// \param[in] _checked True if the menu item is checked
      private slots: void OnEdit(bool _checked);

      /// \brief Contains all the terrain editor tools.
      private: TerrainEditorPalette *terrainPalette;
    };
  }
}
#endif
