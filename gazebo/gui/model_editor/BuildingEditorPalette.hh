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

#ifndef _BUILDING_EDITOR_PALETTE_HH_
#define _BUILDING_EDITOR_PALETTE_HH_

#include <string>
#include <vector>
#include "gazebo/gui/qt.h"
#include "gazebo/common/Events.hh"

namespace gazebo
{
  namespace gui
  {
    /// \addtogroup gazebo_gui
    /// \{

    /// \class BuildingEditorPalette BuildingEditorPalette.hh
    /// \brief A palette that displays different drawing operations.
    class BuildingEditorPalette : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent QWidget.
      public: BuildingEditorPalette(QWidget *_parent = 0);

      /// \brief Destructor
      public: ~BuildingEditorPalette();

      /// \brief Qt callback when the draw wall mode is chosen.
      private slots: void OnDrawWall();

      //private slots: void OnImportImage();

      /// \brief Qt callback when the draw window button is pressed.
      private slots: void OnAddWindow();

      /// \brief Qt callback when the draw door butotn is pressed.
      private slots: void OnAddDoor();

      /// \brief Qt callback when the draw stairs button is pressed.
      private slots: void OnAddStairs();

      /// \brief Qt callback when the discard button pressed.
      private slots: void OnDiscard();

      /// \brief Qt callback when the save button is pressed.
      private slots: void OnSave();

      /// \brief Qt callback when the done button is pressed.
      private slots: void OnDone();

      /// \brief Callback when user has provided information on where
      /// to save the model to.
      /// \param[in] _saveName Name of model.
      /// \param[in] _saveLocation Location to save the model to.
      private: void OnSaveModel(std::string _saveName,
          std::string _saveLocation);

      /// \brief Callback when user confirms to discard model.
      private: void OnDiscardModel();

      /// \brief A label that displays the name of the model.
      private: QLabel *modelNameLabel;

      /// \brief Save button.
      private: QPushButton *saveButton;

      /// \brief Name of model.
      private: std::string modelName;

      /// \brief Save location.
      private: std::string saveLocation;

      /// \brief A list of gui editor events.
      private: std::vector<event::ConnectionPtr> connections;
    };
    /// \}
  }
}

#endif
