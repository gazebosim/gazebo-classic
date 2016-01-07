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
#ifndef _GAZEBO_VIEW_ANGLE_WIDGET_PRIVATE_HH_
#define _GAZEBO_VIEW_ANGLE_WIDGET_PRIVATE_HH_

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    /// \internal
    /// \brief Private data for the ViewAngleWidget class
    class ViewAngleWidgetPrivate
    {
      /// \brief Button for top view.
      public: QToolButton *topButton;

      /// \brief Button for bottom view.
      public: QToolButton *bottomButton;

      /// \brief Button for front view.
      public: QToolButton *frontButton;

      /// \brief Button for back view.
      public: QToolButton *backButton;

      /// \brief Button for left view.
      public: QToolButton *leftButton;

      /// \brief Button for right view.
      public: QToolButton *rightButton;

      /// \brief Button for reset view.
      public: QToolButton *resetButton;

      /// \brief Dropdown menu for projection types.
      public: QComboBox *projectionComboBox;

      /// \brief Main layout.
      public: QGridLayout *mainLayout;

      /// \brief Pointer to the main window.
      public: MainWindow *mainWindow;
    };
  }
}
#endif
