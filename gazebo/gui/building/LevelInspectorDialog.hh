/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include <vector>
#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"
#include "gazebo/gui/building/BaseInspectorDialog.hh"

namespace gazebo
{
  namespace gui
  {
    /// \addtogroup gazebo_gui
    /// \{

    /// \class LevelInspectorDialog LevelInspectorDialog.hh
    /// \brief Dialog for configuring a building level
    class GZ_GUI_BUILDING_VISIBLE LevelInspectorDialog
      : public BaseInspectorDialog
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent QWidget.
      public: LevelInspectorDialog(QWidget *_parent = 0);

      /// \brief Destructor
      public: ~LevelInspectorDialog();

      /// \brief Get the name of the level.
      /// \return The level name.
      public: std::string GetLevelName() const;

      /// \brief Get the height of the level.
      /// \return The level height in pixels.
      public: double GetHeight() const;

      /// \brief Set the name of the level.
      /// \param[in] _levelName New level name.
      public: void SetLevelName(const std::string &_levelName);

      /// \brief Set the height of the level.
      /// \param[in] _height Level height in pixels.
      public: void SetHeight(double _height);

      /// \brief Qt signal emitted to indicate that changes should be applied.
      Q_SIGNALS: void Applied();

      /// \brief Qt callback when the Cancel button is pressed.
      private slots: void OnCancel();

      /// \brief Qt callback when the Apply button is pressed.
      private slots: void OnApply();

      /// \brief Qt callback when the Ok button is pressed.
      private slots: void OnOK();

      /// \brief Widget containing the floor specs.
      public: QWidget *floorWidget;

      /// \brief Editable line that holds the the level name.
      private: QLineEdit *levelNameLineEdit;

      /// \brief Spin box for configuring the level height.
      private: QDoubleSpinBox *heightSpinBox;

      /// \brief Spin box for configuring the floor thickness.
      private: QDoubleSpinBox *floorThicknessSpinBox;
    };
    /// \}
  }
}

#endif
