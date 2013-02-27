/*
 * Copyright 2012 Open Source Robotics Foundation
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

#ifndef _TERRAIN_EDITOR_PALETTE_HH_
#define _TERRAIN_EDITOR_PALETTE_HH_

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    /// \addtogroup gazebo_gui
    /// \{

    /// \class TerrainEditorPalette TerrainEditorPalette.hh
    /// \brief A palette of building items which can be added to the editor.
    class TerrainEditorPalette : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent QWidget.
      public: TerrainEditorPalette(QWidget *_parent = 0);

      /// \brief Destructor
      public: ~TerrainEditorPalette();

      private slots: void OnRaise();

      private slots: void OnLower();

      private slots: void OnSave();
    };
  }
}
#endif
