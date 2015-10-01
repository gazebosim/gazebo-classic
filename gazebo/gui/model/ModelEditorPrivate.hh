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

#ifndef _GAZEBO_MODEL_EDITOR_PRIVATE_HH_
#define _GAZEBO_MODEL_EDITOR_PRIVATE_HH_

#include <string>

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class SchematicViewWidget;
    class ModelTreeWidget;
    class ModelEditorPalette;

    /// \internal
    /// \class ModelEditor ModelEditor.hh
    /// \brief Private data for the ModelEditor class.
    class ModelEditorPrivate
    {
      /// \brief Menubar containing actions related to the editor.
      public: QMenuBar *menuBar;

      /// \brief A palette of entities that can be inserted into the editor.
      public: ModelEditorPalette *modelPalette;

      /// \brief A display of model settings and its child entities
      public: ModelTreeWidget *modelTree;

      /// \brief True if model editor is active.
      public: bool active;

      /// \brief Qt action for adding a previously selected joint in the
      /// model editor.
      public: QAction *jointAct;

      /// \brief Qt signal mapper for mapping add jointsignals.
      public: QSignalMapper *signalMapper;

      /// \brief Previously selected joint type.
      public: std::string selectedJointType;

      /// \brief Action to save model.
      public: QAction *saveAct;

      /// \brief Action to save model as.
      public: QAction *saveAsAct;

      /// \brief Action to start a new model.
      public: QAction *newAct;

      /// \brief Action to exit the editor.
      public: QAction *exitAct;

      /// \brief Action to show joints.
      public: QAction *showJointsAct;

      /// \brief Action to show/hide the schematic view.
      public: QAction *schematicViewAct;

      /// \brief Pointer to the schematic view widget.
      public: SchematicViewWidget *svWidget;

      /// \brief Pointer to the Insert model widget in main window.
      public: QWidget *insertModel;

      /// \brief Show toolbars action cloned from main window.
      public: QAction *showToolbarsAct;

      /// \brief Fullscreen action cloned from main window.
      public: QAction *fullScreenAct;

      /// \brief Camera orthographic view action cloned from main window.
      public: QAction *cameraOrthoAct;

      /// \brief Camera perspective view action cloned from main window.
      public: QAction *cameraPerspectiveAct;

      /// \brief A list of event connections.
      public: std::vector<event::ConnectionPtr> connections;
    };
  }
}
#endif
