/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_GUI_BUILDING_LEVELINSPECTORDIALOG_HH_
#define _GAZEBO_GUI_BUILDING_LEVELINSPECTORDIALOG_HH_

#include <memory>
#include <string>

#include "gazebo/gui/qt.h"
#include "gazebo/gui/building/BaseInspectorDialog.hh"

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    // Forward declare private data.
    class LevelInspectorDialogPrivate;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class LevelInspectorDialog LevelInspectorDialog.hh
    /// \brief Dialog for configuring a building level
    class GZ_GUI_VISIBLE LevelInspectorDialog
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
      public: std::string LevelName() const;

      /// \brief Get the height of the level.
      /// \return The level height in pixels.
      public: double Height() const;

      /// \brief Set the name of the level.
      /// \param[in] _levelName New level name.
      public: void SetLevelName(const std::string &_levelName);

      /// \brief Set the height of the level.
      /// \param[in] _height Level height in pixels.
      public: void SetHeight(const double _height);

      /// \brief Show or hide the floor widget
      /// \param[in] _show True to show the widget, false to hide.
      public: void ShowFloorWidget(const bool _show);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<LevelInspectorDialogPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
