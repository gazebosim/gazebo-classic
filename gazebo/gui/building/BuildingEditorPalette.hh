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

#ifndef _BUILDING_EDITOR_PALETTE_HH_
#define _BUILDING_EDITOR_PALETTE_HH_

#include <string>
#include <vector>
#include <list>
#include <map>

#include "gazebo/gui/qt.h"
#include "gazebo/common/Events.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    /// \addtogroup gazebo_gui
    /// \{

    /// \class BuildingEditorPalette BuildingEditorPalette.hh
    /// \brief A palette of building items which can be added to the editor.
    class GAZEBO_VISIBLE BuildingEditorPalette : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent QWidget.
      public: BuildingEditorPalette(QWidget *_parent = 0);

      /// \brief Destructor
      public: ~BuildingEditorPalette();

      /// \brief Get model name
      /// \return Model name
      public: std::string GetModelName() const;

      /// \brief Qt callback when the draw wall button is pressed.
      private slots: void OnDrawWall();

      /// \brief Qt callback when the draw window button is pressed.
      private slots: void OnAddWindow();

      /// \brief Qt callback when the draw door button is pressed.
      private slots: void OnAddDoor();

      /// \brief Qt callback when the import image button is pressed.
      private slots: void OnImportImage();

      /// \brief Qt callback when the draw stairs button is pressed.
      private slots: void OnAddStair();

      /// \brief Qt callback when a brush is pressed.
      /// \param[in] _buttonId Id of the button clicked.
      private slots: void OnBrush(int _buttonId);

      /// \brief Qt callback when the Model Name field is changed.
      private slots: void OnNameChanged(const QString &_name);

      /// \brief Callback when user has provided information on where to save
      /// the model to.
      /// \param[in] _saveName Name of model being saved.
      /// \param[in] _saveLocation Location to save the model to.
      private: void OnSaveModel(const std::string &_saveName,
          const std::string &_saveLocation);

      /// \brief Event received when an editor item is selected.
      /// \param[in] _mode Type of item to add or empty for none.
      private: void OnCreateEditorItem(const std::string &_mode);

      /// \brief Event received when the user starts a new building model.
      private: void OnNewModel();

      /// \brief Qt callback when the palette is pressed.
      /// \param[in] _event Event.
      private: void mousePressEvent(QMouseEvent *_event);

      /// \brief When a default color button is selected.
      /// \param[in] _buttonId Id of the button clicked.
      private: void OnDefaultColor(int _buttonId);

      /// \brief When the custom color button is selected,
      /// a QColorDialog is opened.
      private: void OnCustomColor();

      /// \brief When any color is selected.
      /// \param[in] _color Color selected.
      private: void OnColor(QColor _color);

      /// \brief When a default texture button is selected.
      /// \param[in] _buttonId Id of the button clicked.
      private: void OnTexture(int _buttonId);

      /// \brief Default name of the building model.
      private: std::string buildingDefaultName;

      /// \brief Edit the name of the building model.
      private: QLineEdit *modelNameEdit;

      /// \brief All the brushes (wall, door, window, stair, etc).
      private: QButtonGroup *brushes;

      /// \brief Link each button ID to a draw mode.
      private: std::map<std::string, int> brushIdToModeMap;

      /// \brief A list of gui editor events connected to this palette.
      private: std::vector<event::ConnectionPtr> connections;

      /// \brief The current draw mode, empty for none.
      private: std::string currentMode;

      /// \brief List of default colors to be picked.
      private: std::vector<QColor> colorList;

      /// \brief List of default textures to be picked.
      private: std::vector<QString> textureList;

      /// \brief Name of the last default color mode.
      private: std::string lastDefaultColor;

      /// \brief Name of the last default texture mode.
      private: std::string lastDefaultTexture;

      /// \brief Custom color button.
      private: QPushButton *customColorButton;
    };
    /// \}
  }
}

#endif
