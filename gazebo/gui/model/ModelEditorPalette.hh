/*
 * Copyright (C) 2013-2015 Open Source Robotics Foundation
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

#ifndef _GAZEBO_MODEL_EDITOR_PALETTE_HH_
#define _GAZEBO_MODEL_EDITOR_PALETTE_HH_

#include <mutex>
#include <map>
#include <string>
#include <vector>

#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/common/Event.hh"

#include "gazebo/gui/model/ModelCreator.hh"
#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
  }

  namespace gui
  {
    class ModelCreator;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class ModelEditorPalette ModelEditorPalette.hh
    /// \brief A palette of model items which can be added to the editor.
    class GZ_GUI_VISIBLE ModelEditorPalette : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent QWidget.
      public: ModelEditorPalette(QWidget *_parent = 0);

      /// \brief Destructor
      public: ~ModelEditorPalette();

      /// \brief Add an item to the model editor palette.
      /// \param[in] _Item item to add.
      /// \param[in] _category Category to add the item too.
      public: void AddItem(QWidget *_item,
          const std::string &_category = "Other");

      /// \brief Add a joint to the model.
      /// \param[in] _type Type of joint to add.
      public: void CreateJoint(const std::string &_type);

      /// \brief Get the model creator.
      /// \return a pointer to the model creator.
      public: ModelCreator *GetModelCreator();

      /// \brief Key event filter callback when key is pressed.
      /// \param[in] _event The key event.
      /// \return True if the event was handled
      private: bool OnKeyPress(const common::KeyEvent &_event);

      /// \brief Callback when an entity is selected.
      /// \param[in] _name Name of entity.
      /// \param[in] _mode Select mode
      private: void OnSetSelectedEntity(const std::string &_name,
          const std::string &_mode);

      /// \brief Callback when a nested model is selected.
      /// \param[in] _name Name of nested model.
      /// \param[in] _selected True if the nested model is selected, false if
      /// deselected.
      private: void OnSetSelectedNestedModel(const std::string &_name,
          bool _selected);

      /// \brief Callback when a link is selected.
      /// \param[in] _name Name of link.
      /// \param[in] _selected True if the link is selected, false if
      /// deselected.
      private: void OnSetSelectedLink(const std::string &_name, bool _selected);

      /// \brief Callback when a joint is selected.
      /// \param[in] _name Name of joint.
      /// \param[in] _selected True if the joint is selected, false if
      /// deselected.
      private: void OnSetSelectedJoint(const std::string &_name,
          bool _selected);

      /// \brief Helper function to deselect a link or a joint.
      /// \param[in] _type Type: Link or Joint.
      private: void DeselectType(const std::string &_type);

      /// \brief Qt callback when cylinder button is clicked.
      private slots: void OnCylinder();

      /// \brief Qt callback when sphere button is clicked.
      private slots: void OnSphere();

      /// \brief Qt callback when box button is clicked.
      private slots: void OnBox();

      /// \brief Qt callback when custom button is clicked.
      private slots: void OnCustom();

      /// \brief Qt callback when a link has been added.
      private slots: void OnLinkAdded();

      /// \brief Qt callback when the model is to be made static.
      private slots: void OnStatic();

      /// \brief Qt callback when the model is allowed to auto disable at rest.
      private slots: void OnAutoDisable();

      /// \brief Qt callback when the Model Name field is changed.
      /// \param[in] _name New name.
      private slots: void OnNameChanged(const QString &_name);

      /// \brief Qt callback when a tree item has been double clicked.
      /// \param[in] _item Item clicked.
      /// \param[in] _column Column index.
      private slots: void OnItemDoubleClicked(QTreeWidgetItem *_item,
          int _column);

      /// \brief Qt callback when a tree item has been clicked.
      /// \param[in] _item Item clicked.
      /// \param[in] _column Column index.
      private slots: void OnItemClicked(QTreeWidgetItem *_item, int _column);

      /// \brief Qt callback when selected items have changed.
      private slots: void OnItemSelectionChanged();

      /// \brief Qt callback when the context menu signal is triggered.
      /// \param[in] _pt Position of the context menu event that the widget
      ///  receives.
      private slots: void OnCustomContextMenu(const QPoint &_pt);

      /// \brief Add a nested model to the tree.
      /// \param[in] _nestedModelName Scoped nested model name.
      private: void OnNestedModelInserted(const std::string &_nestedModelName);

      /// \brief Add a link to the tree.
      /// \param[in] _linkName Scoped link name.
      private: void OnLinkInserted(const std::string &_linkName);

      /// \brief Add a joint to the tree.
      /// \param[in] _jointId Unique joint identifying name.
      /// \param[in] _jointName Scoped name which can be changed by the user.
      /// \param[in] _jointName Scoped name of the parent link.
      /// \param[in] _jointName Scoped name of the child link.
      private: void OnJointInserted(const std::string &_jointId,
          const std::string &_jointName, const std::string &_parentName,
          const std::string &_childName);

      /// \brief Add a model plugin to the tree.
      /// \param[in] _modelPluginName Model plugin name.
      private: void OnModelPluginInserted(const std::string &_modelPluginName);

      /// \brief Remove a nested model from the tree.
      /// \param[in] _linkId Unique nested model identifying name.
      private: void OnNestedModelRemoved(const std::string &_nestedModelId);

      /// \brief Remove a link from the tree.
      /// \param[in] _linkId Unique link identifying name.
      private: void OnLinkRemoved(const std::string &_linkId);

      /// \brief Remove a joint from the tree.
      /// \param[in] _jointId Unique joint identifying name.
      private: void OnJointRemoved(const std::string &_jointId);

      /// \brief Remove all links and joints from the tree.
      private: void ClearModelTree();

      /// \brief Update a joint item text in the tree.
      /// \param[in] _jointId Unique joint identifying name.
      /// \param[in] _newJointName New scoped joint name.
      private: void OnJointNameChanged(const std::string &_jointId,
          const std::string &_newJointName);

      /// \brief Callback when user has provided information on where to save
      /// the model to.
      /// \param[in] _saveName Name of model being saved.
      private: void OnSaveModel(const std::string &_saveName);

      /// \brief Event received when the user starts a new model.
      private: void OnNewModel();

      /// \brief Event received when the model properties changed.
      /// \param[in] _static New static property of the model.
      /// \param[in] _autoDisable New allow_auto_disable property of the model.
      /// \param[in] _pose New model pose.
      /// \param[in] _name New name.
      private: void OnModelPropertiesChanged(bool _static, bool _autoDisable,
          const math::Pose &_pose, const std::string &_name);

      /// \brief A list of gui editor events connected to this palette.
      private: std::vector<event::ConnectionPtr> connections;

      /// \brief Links button group.
      private: QButtonGroup *linkButtonGroup;

      /// \brief Model creator.
      private: ModelCreator *modelCreator;

      /// \brief Static checkbox, true to create a static model.
      private: QCheckBox *staticCheck;

      /// \brief Auto disable checkbox, true to allow model to auto-disable at
      /// rest.
      private: QCheckBox *autoDisableCheck;

      /// \brief Default name of the model.
      private: std::string modelDefaultName;

      /// \brief Edit the name of the model.
      private: QLineEdit *modelNameEdit;

      /// \brief The tree holding all links and joints.
      private: QTreeWidget *modelTreeWidget;

      /// \brief Parent item for all nested models.
      private: QTreeWidgetItem *nestedModelsItem;

      /// \brief Parent item for all links.
      private: QTreeWidgetItem *linksItem;

      /// \brief Parent item for all joints.
      private: QTreeWidgetItem *jointsItem;

      /// \brief Parent item for all model plugins.
      private: QTreeWidgetItem *modelPluginsItem;

      /// \brief Mutex to protect updates.
      private: std::recursive_mutex updateMutex;

      /// \brief Keeps track of selected items.
      private: QList<QTreeWidgetItem *> selected;

      /// \brief Layout for other items in the palette.
      private: QVBoxLayout *otherItemsLayout;

      /// \brief Map of categories to their layout
      private: std::map<std::string, QGridLayout *> categories;
    };
  }
}
#endif
