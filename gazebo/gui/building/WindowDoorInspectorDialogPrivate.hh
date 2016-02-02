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
#ifndef _GAZEBO_GUI_BUILDING_WINDOWDOORINSPECTORDIALOGPRIVATE_HH_
#define _GAZEBO_GUI_BUILDING_WINDOWDOORINSPECTORDIALOGPRIVATE_HH_

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    /// \internal
    /// \brief Private data for WindowDoorInspectorDialog
    class WindowDoorInspectorDialogPrivate
    {
      /// \brief Label that displays the name of the item.
      public: QLabel *itemNameLabel;

      /// \brief Spin box for configuring the width of the item.
      public: QDoubleSpinBox *widthSpinBox;

      /// \brief Spin box for configuring the depth of the item.
      public: QDoubleSpinBox *depthSpinBox;

      /// \brief Spin box for configuring the height of the item.
      public: QDoubleSpinBox *heightSpinBox;

      /// \brief Spin box for configuring the X position of the item.
      public: QDoubleSpinBox *positionXSpinBox;

      /// \brief Spin box for configuring the Y position of the item.
      public: QDoubleSpinBox *positionYSpinBox;

      /// \brief Spin box for configuring the elevation of the item.
      public: QDoubleSpinBox *elevationSpinBox;

      /// \brief Combo box for selecting the type of the item to use.
      public: QComboBox *typeComboBox;
    };
  }
}
#endif
