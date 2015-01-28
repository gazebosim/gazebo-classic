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
    class SaveDialogPrivate;

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
                /// \brief Save model
                MODEL,
                /// \brief Save building
                BUILDING
              };

      /// \brief Constructor.
      /// \param[in] _mode Mode of the dialog.
      /// \param[in] _parent Parent QWidget.
      public: SaveDialog(int _mode = 0, QWidget *_parent = 0);

      /// \brief Destructor.
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

      /// \brief Add the parent folder of _path to the model path represented
      /// by SystemPaths, notify InsertModelWidget to display the model name in
      /// the "Insert Models" tab, and write the parent folder filename to
      /// gui.ini
      /// \param[in] _path Path to be added.
      public: void AddDirToModelPaths(const std::string &_path);

       /// \brief Helper function to generate a valid folder name from a
       /// human-readable model name.
       /// \param[in] _modelName Human-readable model name.
       /// \return Folder name.
      public: std::string GetFolderNameFromModelName(const std::string
          &_modelName);

      /// \brief Call to execute the dialog.
      /// \return True if the user accepted the dialog.
      public: bool OnSaveAs();

      /// \brief Qt callback when the file directory browse button is pressed.
      private slots: void OnBrowse();

      /// \brief Qt callback when the Cancel button is pressed.
      private slots: void OnCancel();

      /// \brief Qt callback when the Save button is pressed.
      private slots: void OnAcceptSave();

      /// \brief Get a template config file for a simple model.
      private: std::string GetTemplateConfigString();

      /// \brief Generate the config file.
      public: void GenerateConfig();

      /// \brief Save config file.
      public: void SaveToConfig();

      /// \brief Save model to SDF format.
      /// \param[in] _modelSDF Pointer to the model SDF.
      public: void SaveToSDF(sdf::SDFPtr _modelSDF);

      /// \brief Qt callback to show/hide advanced model saving options.
      /// \param[in] _checked Whether it is checked or not.
      private slots: void ToggleAdvancedOptions(bool _checked);

      /// \internal
      /// \brief Pointer to private data.
      private: SaveDialogPrivate *dataPtr;
    };
    /// \}
  }
}

#endif
