/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_GUI_MODEL_MODELEDITOR_HH_
#define GAZEBO_GUI_MODEL_MODELEDITOR_HH_

#include <memory>
#include <string>

#include <sdf/sdf.hh>

#include "gazebo/gui/qt.h"
#include "gazebo/gui/Editor.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class TopToolbar;
    class ModelEditorPrivate;

    /// \class ModelEditor ModelEditor.hh gui/gui.hh
    /// \brief Interface to the terrain editor.
    class GZ_GUI_VISIBLE ModelEditor : public Editor
    {
      Q_OBJECT

      /// \brief Constuctor.
      /// \param[in] _mainWindow Pointer to the mainwindow.
      public: ModelEditor(MainWindow *_mainWindow);

      /// \brief Destuctor.
      public: virtual ~ModelEditor();

      /// \brief Add an item to palette.
      /// \param[in] _Item item to add.
      /// \param[in] _category Category to add the item too.
      public: void AddItemToPalette(QWidget *_item,
          const std::string &_category = "");

      /// \brief Add an entity to the editor
      /// \param[in] _sdf SDF describing the entity.
      public: void AddEntity(sdf::ElementPtr _sdf);

      /// \brief Qt callback when the model editor's save action is
      /// triggered.
      private slots: void Save();

      /// \brief Qt callback when the model editor's save as action is
      /// triggered.
      private slots: void SaveAs();

      /// \brief Qt callback when the model editor's new action is
      /// triggered.
      private slots: void New();

      /// \brief Qt callback when the model editor's exit action is
      /// triggered.
      private slots: void Exit();

      /// \brief QT callback when entering model edit mode
      /// \param[in] _checked True if the menu item is checked
      private slots: void OnEdit(bool _checked);

      /// \brief QT callback when the joint button is clicked.
      private slots: void OnAddSelectedJoint();

      /// \brief QT callback when a joint menu is selected
      /// \param[in] _type Type of joint.
      private slots: void OnAddJoint(const QString &_type);

      /// \brief Qt callback when a joint is added.
      private slots: void OnJointAdded();

      /// \brief Show the schematic view widget
      /// \param[in] _show True to show the widget, false to hide it.
      private slots: void OnSchematicView(bool _show);

      /// \brief Callback when the model has been completed.
      private: void OnFinish();

      /// \brief Toggle the model material scheme. This greys out all
      /// non-editable models when the user enters the model editor mode.
      private: void ToggleMaterialScheme();

      /// \brief Toggle main window's toolbar to display model editor icons.
      private: void ToggleToolbar();

      /// \brief Toggle the Insert tab on and off in model editor mode.
      /// This takes the main window Insert tab and adds it to the editor
      /// Note that ToggleToolbar restores the Insert tab in the main window
      private: void ToggleInsertWidget();

      /// \brief Create menus
      private: void CreateMenus();

      /// \brief Event callback used for inserting models into the editor
      /// \param[in] _type Type of entity to be inserted
      /// \param[in] _data Event data (e.g. name of model).
      private: void OnCreateEntity(const std::string &_type,
                                   const std::string &_data);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<ModelEditorPrivate> dataPtr;
    };
  }
}
#endif
