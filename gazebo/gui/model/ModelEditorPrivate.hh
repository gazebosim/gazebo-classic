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

#ifndef _MODEL_EDITOR_PRIVATE_HH_
#define _MODEL_EDITOR_PRIVATE_HH_

#include <string>

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class ModelEditorPalette;

    /// \class ModelEditor ModelEditor.hh
    /// \brief Private data for the ModelEditor class.
    class ModelEditorPrivate
    {
      /// \brief Menubar containing actions related to the editor.
      public: QMenuBar *menuBar;

      /// \brief Contains all the model editor tools.
      public: ModelEditorPalette *modelPalette;

      /// \brief True if model editor is active.
      public: bool active;

      /// \brief Qt action for selecting and adding a joint in the model editor.
      public: QAction *jointTypeAct;

      /// \brief Qt action for adding a previously selected joint in the
      /// model editor.
      public: QAction *jointAct;

      /// \brief A separator for the joint icon.
      public: QAction *jointSeparatorAct;

      /// \brief Qt tool button associated with the joint action.
      public: QToolButton *jointButton;

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

      /// \brief Save the main window paused state to use when returning.
      public: bool mainWindowPaused;
    };
  }
}
#endif
