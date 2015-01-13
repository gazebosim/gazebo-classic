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

#ifndef _SAVE_DIALOG_HH_
#define _SAVE_DIALOG_HH_

#include <string>
#include <sdf/sdf.hh>
#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    /// \addtogroup gazebo_gui
    /// \{

    /// \class SaveDialog SaveDialog.hh gui/gui.hh
    /// \brief Dialog for saving to file.
    class GAZEBO_VISIBLE SaveDialog : public QDialog
    {
      Q_OBJECT

      /// \enum SaveMode
      /// \brief Unique identifiers for all dialog modes.
      public: enum SaveMode {
                /// \brief Save building
                BUILDING,
                /// \brief Save model
                MODEL
              };

      /// \brief Constructor
      /// \param[in] _mode Mode of the dialog
      /// \param[in] _parent Parent QWidget.
      public: SaveDialog(int _mode = 0, QWidget *_parent = 0);

      /// \brief Destructor
      public: ~SaveDialog();

      /// \brief Get the model name.
      /// \return The model name.
      public: std::string GetModelName() const;

      /// \brief Get the save location.
      /// \return Path of the save location.
      public: std::string GetSaveLocation() const;

      /// \brief Set the model name.
      /// \param[in] _name Name to set the model to.
      public: void SetModelName(const std::string &_name);

      /// \brief Set the save location.
      /// \param[in] _location Location to save to.
      public: void SetSaveLocation(const std::string &_location);

      /// \brief Get the model's author's name.
      /// \return The author's name.
      public: std::string GetAuthorName() const;

      /// \brief Get the model's author's email.
      /// \return The author's email.
      public: std::string GetAuthorEmail() const;

      /// \brief Get the model's description.
      /// \return The model's description.
      public: std::string GetDescription() const;

      /// \brief Get the model's version.
      /// \return The model's version.
      public: std::string GetVersion() const;

      /// \brief TODO
      public: void AddDirToModelPaths(const std::string& _path);

      /// \brief Qt callback when the file directory browse button is pressed.
      private slots: void OnBrowse();

      /// \brief Qt callback when the Cancel button is pressed.
      private slots: void OnCancel();

      /// \brief Qt callback when the Save button is pressed.
      private slots: void OnSave();

      /// \brief Callback for selecting a folder and saving the model.
      /// \param[in] _saveName Name to save the model.
      /// \return True if the user chose to save, false if the user cancelled.
      public: bool OnSaveAs(const std::string &_saveName);

      /// \brief Get a template config file for a simple model.
      private: std::string GetTemplateConfigString();

      /// \brief Generate the config file.
      public: void GenerateConfig();

      /// \brief Save config file.
      /// \param[in] _savePath Path to save the file to.
      public: void SaveToConfig(const std::string &_savePath);

      /// \brief Qt callback to show/hide advanced model saving options.
      private slots: void ToggleAdvancedOptions(bool _checked);

      /// \brief Widget container to hold advanced model saving options.
      private: QWidget *advancedOptionsWidget;

      /// \brief Label appearing at the top of the dialog box.
      private: QLabel *messageLabel;

      /// \brief Editable line that holds the model name.
      private: QLineEdit* modelNameLineEdit;

      /// \brief Editable line that holds the model's version.
      private: QLineEdit* modelVersionLineEdit;

      /// \brief Editable line that holds the model's description.
      private: QLineEdit* modelDescriptionLineEdit;

      /// \brief Editable line that holds the model's author's name.
      private: QLineEdit* modelAuthorNameLineEdit;

      /// \brief Editable line that holds the model's author's email.
      private: QLineEdit* modelAuthorEmailLineEdit;

      /// \brief Editable line that holds the model's save location.
      private: QLineEdit* modelLocationLineEdit;

      /// \brief The model's config file.
      private: TiXmlDocument modelConfig;
    };
    /// \}
  }
}

#endif
