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

#ifndef _GAZEBO_GUI_LEVEL_WIDGET_PRIVATE_HH_
#define _GAZEBO_GUI_LEVEL_WIDGET_PRIVATE_HH_

#include <vector>

#include "gazebo/common/CommonTypes.hh"

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    /// \internal
    /// \brief Private data for the LevelWidget class
    class LevelWidgetPrivate
    {
      /// \brief Combo box for selecting the current level.
      public: QComboBox *levelComboBox;

      /// \brief A list of gui editor events connected to this widget
      public: std::vector<event::ConnectionPtr> connections;

      /// \brief Counter for the total number of levels.
      public: int levelCounter;

      /// \brief Action to show floorplan.
      public: QAction *showFloorplanAct;

      /// \brief Action to show elements.
      public: QAction *showElementsAct;
    };
  }
}

#endif
