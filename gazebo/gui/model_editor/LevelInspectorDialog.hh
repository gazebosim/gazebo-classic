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

#ifndef _LEVEL_INSPECTOR_DIALOG_HH_
#define _LEVEL_INSPECTOR_DIALOG_HH_

#include <string>
#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class LevelInspectorDialog : public QDialog
    {
      Q_OBJECT

      public: enum type {WINDOW, DOOR};

      public: LevelInspectorDialog(QWidget *_parent = 0);

      public: ~LevelInspectorDialog();

      public: std::string GetLevelName() const;

      public: double GetHeight() const;

      public: void SetLevelName(const std::string &_levelName);

      public: void SetHeight(double _height);

      Q_SIGNALS: void Applied();

      private slots: void OnCancel();

      private slots: void OnApply();

      private slots: void OnOK();

      private: QLineEdit* levelNameLineEdit;

      private: QDoubleSpinBox *heightSpinBox;

      private: QDoubleSpinBox *floorThicknessSpinBox;

      private: QComboBox *materialComboBox;
    };
  }
}

#endif
