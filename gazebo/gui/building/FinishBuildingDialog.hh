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

#ifndef _FINISH_BUILDING_DIALOG_HH_
#define _FINISH_BUILDING_DIALOG_HH_

#include <string>
#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    /// \addtogroup gazebo_gui
    /// \{

    /// \class FinishBuildingDialog FinishBuildingDialog.hh
    /// \brief Dialog for saving the building model.
    class GZ_GUI_BUILDING_VISIBLE FinishBuildingDialog : public QDialog
    {
      Q_OBJECT

      /// \enum FinishMode
      /// \brief Unique identifiers for all dialog modes.
      public: enum FinishMode {
                /// \brief Finish mode
                MODEL_FINISH,
                /// \brief Save mode
                MODEL_SAVE
              };

      /// \brief Constructor
      /// \param[in] _mode Mode of the dialog
      /// \param[in] _parent Parent QWidget.
      public: FinishBuildingDialog(int _mode = 0, QWidget *_parent = 0);

      /// \brief Destructor
      public: ~FinishBuildingDialog();

      /// \brief Get the building model name.
      /// \return The model name.
      public: std::string GetModelName() const;

      /// \brief Get the save location.
      /// \return Path of the save location.
      public: std::string GetSaveLocation() const;

      /// \brief Set the building model name.
      /// \param[in] _name Name to set the model to.
      public: void SetModelName(const std::string &_name);

      /// \brief Set the save location.
      /// \param[in] _location Location to save to.
      public: void SetSaveLocation(const std::string &_location);

      /// \brief Qt callback when the file directory browse button is pressed.
      private slots: void OnBrowse();

      /// \brief Qt callback when the Cancel button is pressed.
      private slots: void OnCancel();

      /// \brief Qt callback when the Done button is pressed.
      private slots: void OnFinish();

      /// \brief Editable line that holds the model name.
      private: QLineEdit* modelNameLineEdit;

      /// \brief Editable line that holds the model's save location.
      private: QLineEdit* modelLocationLineEdit;
    };
    /// \}
  }
}

#endif
