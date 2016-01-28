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
#ifndef _GAZEBO_GUI_BUILDING_WALLINSPECTORDIALOGPRIVATE_HH_
#define _GAZEBO_GUI_BUILDING_WALLINSPECTORDIALOGPRIVATE_HH_

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    /// \internal
    /// \brief Private data for WallInspectorDialog
    class WallInspectorDialogPrivate
    {
      /// \brief Spin box for configuring the X start position of the wall
      /// segment.
      public: QDoubleSpinBox *startXSpinBox;

      /// \brief Spin box for configuring the Y start position of the wall
      /// segment.
      public: QDoubleSpinBox *startYSpinBox;

      /// \brief Spin box for configuring the X end position of the wall
      /// segment.
      public: QDoubleSpinBox *endXSpinBox;

      /// \brief Spin box for configuring the Y end position of the wall
      /// segment.
      public: QDoubleSpinBox *endYSpinBox;

      /// \brief Spin box for configuring the height of the wall.
      public: QDoubleSpinBox *heightSpinBox;

      /// \brief Spin box for configuring the thickness of the wall.
      public: QDoubleSpinBox *thicknessSpinBox;

      /// \brief Spin box for configuring the length of the wall segment.
      public: QDoubleSpinBox *lengthSpinBox;

      /// \brief Label that holds the name of the wall.
      public: QLabel *wallNameLabel;
    };
  }
}
#endif
