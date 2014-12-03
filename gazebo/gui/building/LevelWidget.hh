/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#ifndef _LEVEL_WIDGET_HH_
#define _LEVEL_WIDGET_HH_

#include <string>
#include <vector>
#include "gazebo/gui/qt.h"
#include "gazebo/common/Event.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    /// \addtogroup gazebo_gui
    /// \{

    /// \class GridLines GridLines.hh
    /// \brief A widget for adding and changing building levels.
    class GZ_GUI_BUILDING_VISIBLE LevelWidget : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent QWidget.
      public: LevelWidget(QWidget *_parent = 0);

      /// \brief Destructor
      public: ~LevelWidget();

      /// \brief Qt callback when the selection of the level combo box has been
      /// changed.
      public slots: void OnCurrentLevelChanged(int _level);

      /// \brief Qt callback when the add level button has been pressed.
      public slots: void OnAddLevel();

      /// \brief Qt callback when the delete level button has been pressed.
      public slots: void OnDeleteLevel();

      /// \brief Qt callback when the show floorplan button has been pressed.
      public slots: void OnShowFloorplan();

      /// \brief Trigger show floorplan.
      public slots: void OnTriggerShowFloorplan();

      /// \brief Qt callback when the show elements button has been pressed.
      public slots: void OnShowElements();

      /// \brief Trigger show elements.
      public slots: void OnTriggerShowElements();

      /// \brief Callback received when levels are changed externally.
      private: void OnUpdateLevelWidget(int _level,
          const std::string &_newName);

      /// \brief Callback received when the widget must be reset.
      private: void OnDiscard();

      /// \brief Combo box for selecting the current level.
      private: QComboBox *levelComboBox;

      /// \brief A list of gui editor events connected to this widget
      private: std::vector<event::ConnectionPtr> connections;

      /// \brief Counter for the total number of levels.
      private: int levelCounter;

      /// \brief Action to show floorplan.
      private: QAction *showFloorplanAct;

      /// \brief Action to show elements.
      private: QAction *showElementsAct;
    };
    /// \}
  }
}

#endif
