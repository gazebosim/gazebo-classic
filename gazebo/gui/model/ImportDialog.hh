/*
 * Copyright 2013 Open Source Robotics Foundation
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

#ifndef _MODEL_IMPORT_DIALOG_HH_
#define _MODEL_IMPORT_DIALOG_HH_

#include <string>
#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    /// \addtogroup gazebo_gui
    /// \{

    /// \class ImportDialog gui/ImportDialog.hh
    /// \brief Dialog for saving to file.
    class ImportDialog : public QDialog
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent QWidget.
      public: ImportDialog(QWidget *_parent = 0);

      /// \brief Destructor
      public: ~ImportDialog();

      /// \brief Get name of file.
      /// \return The name of file.
      public: std::string GetPartName() const;

      /// \brief Get the import path of the custom part.
      /// \return Path of the custom location.
      public: std::string GetImportPath() const;

      /// \brief Set the name of the custom part.
      /// \param[in] _name Name of file.
      public: void SetPartName(const std::string &_name);

      /// \brief Set the import path of the custom part.
      /// \param[in] _path Path to import the custom part.
      public: void SetImportPath(const std::string &_path);

      /// \brief Set the message to be displayed.
      /// \param[in] _msg Message to be displayed.
      public: void SetMessage(const std::string &_msg);

      /// \brief Set the tile of the dialog.
      /// \param[in] _title Title of dialog.
      public: void SetTitle(const std::string &_title);

      /// \brief Qt event emitted showing the dialog
      protected: virtual void showEvent(QShowEvent *event);

      /// \brief Qt callback when the file directory browse button is pressed.
      private slots: void OnBrowse();

      /// \brief Qt callback when the Cancel button is pressed.
      private slots: void OnCancel();

      /// \brief Qt callback when the import button is pressed.
      private slots: void OnImport();

      /// \brief Editable line that holds the name.
      private: QLineEdit *nameLineEdit;

      /// \brief Editable line that holds the import location.
      private: QLineEdit *pathLineEdit;

      /// \brief Message displayed in the dialog.
      private: QLabel *messageLabel;
    };
    /// \}
  }
}

#endif
