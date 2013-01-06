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

#ifndef _LEVEL_WIDGET_HH_
#define _LEVEL_WIDGET_HH_

#include "gazebo/gui/qt.h"
#include "gazebo/common/Event.hh"

namespace gazebo
{
  namespace gui
  {
    class LevelWidget : public QWidget
    {
      Q_OBJECT

      public: LevelWidget(QWidget *_parent = 0);

      public: ~LevelWidget();

      signals: void LevelChanged(int _level);

      signals: void LevelAdded();

      public slots: void OnCurrentLevelChanged(int _level);

      public slots: void OnAddLevel();

      private: void OnChangeLevelName(int _level, const std::string &_newName);

      private: void OnDiscard();

      private: QComboBox *levelComboBox;

      private: std::vector<event::ConnectionPtr> connections;
    };
  }
}

#endif
