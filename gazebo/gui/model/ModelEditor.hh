/*
 * Copyright 2013 Open Source Robotics Foundation
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
#ifndef _MODEL_EDITOR_HH_
#define _MODEL_EDITOR_HH_

#include "gazebo/gui/qt.h"
#include "gazebo/gui/Editor.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class ModelEditorPalette;

    /// \class ModelEditor ModelEditor.hh gui/gui.hh
    /// \brief Interface to the terrain editor.
    class GAZEBO_VISIBLE ModelEditor : public Editor
    {
      Q_OBJECT

      /// \brief Constuctor.
      /// \param[in] _mainWindow Pointer to the mainwindow.
      public: ModelEditor(MainWindow *_mainWindow);

      /// \brief Destuctor.
      public: virtual ~ModelEditor();

      /// \brief QT callback when entering model edit mode
      /// \param[in] _checked True if the menu item is checked
      private slots: void OnEdit(bool _checked);

      /// \brief Callback when the model has been completed.
      private: void OnFinish();

      /// \brief Toggle main window's toolbar to display model editor icons.
      private: void ToggleToolbar();

      /// \brief Contains all the model editor tools.
      private: ModelEditorPalette *modelPalette;

      /// \brief True if model editor is active.
      private: bool active;
    };
  }
}
#endif
