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

#ifndef GAZEBO_GUI_MODEL_MODELCREATOR_HH_
#define GAZEBO_GUI_MODEL_MODELCREATOR_HH_

#include <map>
#include <memory>
#include <mutex>
#include <string>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>

#include "gazebo/gui/qt.h"

#include "gazebo/rendering/RenderTypes.hh"

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    class KeyEvent;
    class MouseEvent;
  }

  namespace gui
  {
    class JointMaker;
    class LinkData;
    class ModelPluginData;
    class NestedModelData;

    // Forward declare private data.
    class ModelCreatorPrivate;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class ModelCreator ModelCreator.hh
    /// \brief Create and manage 3D visuals of a model with links, nested models
    /// and joints.
    class GZ_GUI_VISIBLE ModelCreator : public QObject
    {
      Q_OBJECT

      /// \enum Entity types
      /// \brief Unique identifiers for entity types that can be created.
      public: enum EntityType
      {
        /// \brief none
        ENTITY_NONE,
        /// \brief Box
        ENTITY_BOX,
        /// \brief Sphere
        ENTITY_SPHERE,
        /// \brief Cylinder
        ENTITY_CYLINDER,
        /// \brief Imported 3D mesh
        ENTITY_MESH,
        /// \brief Extruded polyline
        ENTITY_POLYLINE,
        /// \brief Nested model
        ENTITY_MODEL
      };

      /// \enum SaveState
      /// \brief Save states for the model editor.
      public: enum SaveState
      {
        /// \brief The model has never been saved.
        NEVER_SAVED,

        /// \brief All changes have been saved.
        ALL_SAVED,

        /// \brief Has been saved before, but has unsaved changes.
        UNSAVED_CHANGES
      };

      /// \brief Constructor
      /// \param[in] _parent Parent Qt Object
      public: ModelCreator(QObject *_parent = NULL);

      /// \brief Destructor
      public: virtual ~ModelCreator();

      /// \brief Enable the mouse and key event handlers.
      public: void EnableEventHandlers();

      /// \brief Disable the mouse and key event handlers.
      public: void DisableEventHandlers();

      /// \brief Set the name of the model.
      /// \param[in] _modelName Name of the model to set to.
      public: void SetModelName(const std::string &_modelName);

      /// \brief Get the name of the model.
      /// \return Name of model.
      public: std::string ModelName() const;

      /// \brief Set save state upon a change to the model.
      public: void ModelChanged();

      /// \brief Helper function to manage writing files to disk.
      public: void SaveModelFiles();

      /// \brief Finish the model and create the entity on the gzserver.
      public: void FinishModel();

      /// \brief Begin the process of inserting a custom link using the mouse.
      /// \param[in] _type Type of link to add: ENTITY_BOX, ENTITY_CYLINDER,
      /// ENTITY_SPHERE, ENTITY_MESH or ENTITY_POLYLINE.
      /// \param[in] _size Size of the link.
      /// \param[in] _pose Pose of the link.
      /// \param[in] _samples Number of samples for polyline.
      public: void AddCustomLink(const EntityType _type,
          const ignition::math::Vector3d &_size = ignition::math::Vector3d::One,
          const ignition::math::Pose3d &_pose = ignition::math::Pose3d::Zero,
          const std::string &_uri = "", const unsigned int _samples = 5);

      /// \brief Add a link to the model.
      /// \param[in] _type Type of link to add: ENTITY_BOX, ENTITY_CYLINDER,
      /// ENTITY_SPHERE, ENTITY_MESH or ENTITY_POLYLINE.
      /// \param[in] _size Size of the link.
      /// \param[in] _pose Pose of the link.
      /// \param[in] _samples Number of samples for polyline.
      /// \return Link data.
      public: LinkData *AddShape(const EntityType _type,
          const ignition::math::Vector3d &_size = ignition::math::Vector3d::One,
          const ignition::math::Pose3d &_pose = ignition::math::Pose3d::Zero,
          const std::string &_uri = "", const unsigned int _samples = 5);

      /// \brief Add a nested model to the model
      /// \param[in] _sdf SDF describing the model.
      /// \return Nested model data.
      public: NestedModelData *AddModel(const sdf::ElementPtr &_sdf);

      /// \brief Add a joint to the model.
      /// \param[in] _type Type of joint to add.
      public: void AddJoint(const std::string &_type);

      /// \brief Remove an entity from the model.
      /// \param[in] _entityName Name of the entity to remove
      public: void RemoveEntity(const std::string &_entityName);

      /// \brief Remove a model plugin from the model.
      /// \param[in] _pluginName Name of the model plugin to remove.
      /// \param[in] _newCmd Flag indicating whether a new command should be
      /// created.
      public: void RemoveModelPlugin(const std::string &_pluginName,
          const bool _newCmd = true);

      /// \brief Set the model to be static
      /// \param[in] _static True to make the model static.
      public: void SetStatic(const bool _static);

      /// \brief Set the model to allow auto disable at rest.
      /// \param[in] _auto True to allow the model to auto disable.
      public: void SetAutoDisable(const bool _auto);

      /// \brief Reset the model creator and the SDF.
      public: void Reset();

      /// \brief Stop the process of adding a link or joint to the model.
      public: void Stop();

      /// \brief Get joint maker
      /// \return Joint maker
      public: gui::JointMaker *JointMaker() const;

      /// \brief Set the select state of an entity.
      /// \param[in] _name Name of the link.
      /// \param[in] _selected True to select the entity.
      public: void SetSelected(const std::string &_name, const bool selected);

      /// \brief Set the select state of a entity visual.
      /// \param[in] _linkVis Pointer to the entity visual.
      /// \param[in] _selected True to select the entity.
      public: void SetSelected(const rendering::VisualPtr &_entityVis,
          const bool selected);

      /// \brief Get current save state.
      /// \return Current save state.
      public: enum SaveState CurrentSaveState() const;

      /// \brief Add an entity to the model
      /// \param[in] _sdf SDF describing the entity.
      public: void AddEntity(const sdf::ElementPtr &_sdf);

      /// \brief Add a link to the model
      /// \param[in] _type Type of link to be added
      public: void AddLink(const EntityType _type);

      /// \brief Add a model plugin to the model
      /// \param[in] _name Name of plugin
      /// \param[in] _filename Plugin filename
      /// \param[in] _innerxml Plugin SDF elements in string
      /// \param[in] _newCmd Flag indicating whether a new command should be
      /// created.
      public: void OnAddModelPlugin(const std::string &_name,
          const std::string &_filename, const std::string &_innerxml,
          const bool _newCmd = true);

      /// \brief Add a model plugin to the model
      /// \param[in] _pluginElem Pointer to plugin SDF element
      public: void AddModelPlugin(const sdf::ElementPtr &_pluginElem);

      /// \brief Get a model plugin data by its name
      /// \param[in] _name Name of model plugin
      /// \return Model plugin data.
      public: ModelPluginData *ModelPlugin(const std::string &_name);

      /// \brief Generate the SDF from model link and joint visuals.
      public: void GenerateSDF();

      /// \brief Convert a given pose from the world frame to the local frame
      /// of the model being edited.
      /// \param[in] _world Pose in world frame.
      /// \return Pose in model local frame.
      public: ignition::math::Pose3d WorldToLocal(
          const ignition::math::Pose3d &_world) const;

      /// \brief Helper function to generate link sdf from link data.
      /// \param[in] _link Link data used to generate the sdf.
      /// \return SDF element describing the link.
      private: sdf::ElementPtr GenerateLinkSDF(LinkData *_link);

      /// \brief Callback for newing the model.
      private: void OnNew();

      /// \brief Callback for saving the model.
      /// \return True if the user chose to save, false if the user cancelled.
      private: bool OnSave();

      /// \brief Callback for selecting a folder and saving the model.
      /// \return True if the user chose to save, false if the user cancelled.
      private: bool OnSaveAs();

      /// \brief Callback for when the name is changed through the model
      /// settings tab.
      /// \param[in] _modelName The newly entered model name.
      private: void OnNameChanged(const std::string &_modelName);

      /// \brief Event received when the model properties changed.
      /// \param[in] _static New static property of the model.
      /// \param[in] _autoDisable New allow_auto_disable property of the model.
      private: void OnPropertiesChanged(const bool _static,
          const bool _autoDisable);

      /// \brief Callback received when exiting the editor mode.
      private: void OnExit();

      /// \brief Internal helper function to remove a nestedModel without
      /// removing the joints.
      /// \param[in] _nestedModelName Name of the nestedModel to remove
      private: void RemoveNestedModelImpl(const std::string &_nestedModelName);

      /// \brief Internal helper function to remove a link without removing
      /// the joints.
      /// \param[in] _linkName Name of the link to remove
      private: void RemoveLinkImpl(const std::string &_linkName);

      /// \brief QT callback when entering model edit mode
      /// \param[in] _checked True if the menu item is checked
      private slots: void OnEdit(const bool _checked);

      /// \brief QT callback when there's a request to edit an existing model.
      /// \param[in] _modelName Name of model to be edited.
      private slots: void OnEditModel(const std::string &_modelName);

      /// \brief Qt callback when the copy action is triggered.
      private slots: void OnCopy();

      /// \brief Qt callback when the paste action is triggered.
      private slots: void OnPaste();

      /// \brief Mouse event filter callback when mouse is pressed.
      /// \param[in] _event The mouse event.
      /// \return True if the event was handled
      private: bool OnMousePress(const common::MouseEvent &_event);

      /// \brief Mouse event filter callback when mouse is released.
      /// \param[in] _event The mouse event.
      /// \return True if the event was handled
      private: bool OnMouseRelease(const common::MouseEvent &_event);

      /// \brief Mouse event filter callback when mouse is moved.
      /// \param[in] _event The mouse event.
      /// \return True if the event was handled
      private: bool OnMouseMove(const common::MouseEvent &_event);

      /// \brief Mouse event filter callback when mouse is double clicked.
      /// \param[in] _event The mouse event.
      /// \return True if the event was handled
      private: bool OnMouseDoubleClick(const common::MouseEvent &_event);

      /// \brief Key event filter callback when key is pressed.
      /// \param[in] _event The key event.
      /// \return True if the event was handled
      private: bool OnKeyPress(const common::KeyEvent &_event);

      /// \brief Callback when the manipulation mode has changed.
      /// \param[in] _mode New manipulation mode.
      private: void OnManipMode(const std::string &_mode);

      /// \brief Callback when an entity is selected outside of the editor.
      /// \param[in] _name Name of entity.
      /// \param[in] _mode Select mode
      private: void OnDeselectAll(const std::string &_name,
          const std::string &_mode);

      /// \brief Callback when a model editor entity is selected.
      /// \param[in] _name Name of entity.
      /// \param[in] _selected True if the entity is selected, false if
      /// deselected.
      private: void OnSetSelectedEntity(const std::string &_name,
          const bool _selected);

      /// \brief Callback when a model plugin is selected.
      /// \param[in] _name Name of plugin.
      /// \param[in] _selected True if the plugin is selected, false if
      /// deselected.
      private: void OnSetSelectedModelPlugin(const std::string &_name,
          const bool _selected);

      /// \brief Create link with default properties from a visual. This
      /// function creates a link that will become the parent of the
      /// input visual. A collision visual with the same geometry as the input
      /// visual will also be added to the link.
      /// \param[in] _visual Visual used to create the link.
      /// \return Link data.
      private: LinkData *CreateLink(const rendering::VisualPtr &_visual);

      /// \brief Insert a link from an SDF element.
      /// \param[in] _sdf SDF element with link data.
      private: void InsertLinkFromSDF(sdf::ElementPtr _sdf);

      /// \brief Insert a nested model from an SDF element.
      /// \param[in] _sdf SDF element with nested model data.
      private: void InsertNestedModelFromSDF(sdf::ElementPtr _sdf);

      /// \brief Clone an existing nested model.
      /// \param[in] _modelName Name of nested model to be cloned.
      /// \return Cloned nested model data.
      private: NestedModelData *CloneNestedModel(const std::string &_modelName);

      /// \brief Clone an existing link.
      /// \param[in] _linkName Name of link to be cloned.
      /// \return Cloned link data.
      private: LinkData *CloneLink(const std::string &_linkName);

      /// \brief Create a link from an SDF with the specified parent visual.
      /// \param[in] _linkElem SDF element of the link that will be used to
      /// recreate its visual representation in the model editor.
      /// \param[in] _parentVis Parent visual that the link will be attached to.
      /// \return Link data.
      private: LinkData *CreateLinkFromSDF(const sdf::ElementPtr &_linkElem,
          const rendering::VisualPtr &_parentVis);

      /// \brief Open the link inspector.
      /// \param[in] _name Name of link.
      private: void OpenInspector(const std::string &_name);

      /// \brief Open the model plugin inspector.
      /// \param[in] _name Name of model plugin.
      private: void OpenModelPluginInspector(const std::string &_name);

      /// \brief Spawn the entity in simulation.
      private: virtual void CreateTheEntity();

      /// \brief Internal init function.
      private: bool Init();

      /// \brief Create an empty model.
      /// \return Name of the model created.
      private: std::string CreateModel();

      /// \brief Load a model SDF file and create visuals in the model editor.
      /// This is used mainly when editing existing models. The function is
      /// called recursively to create nested models.
      /// \param[in] _sdf SDF of a model to be loaded
      /// \param[in] _parentVis If this is not the root model, it will have a
      /// parent visual for its parent model.
      /// \param[in] _emit True to emit nested model inserted events.
      /// \return Data describing the model.
      private: NestedModelData *CreateModelFromSDF(const sdf::ElementPtr &_sdf,
          const rendering::VisualPtr &_parentVis = NULL,
          const bool _emit = true);

      /// \brief Callback when a specific alignment configuration is set.
      /// \param[in] _axis Axis of alignment: x, y, or z.
      /// \param[in] _config Configuration: min, center, or max.
      /// \param[in] _target Target of alignment: first or last.
      /// \param[in] _preview True to preview alignment without publishing
      /// to server.
      /// \param[in] _inverted True to invert alignment direction.
      private: void OnAlignMode(const std::string &_axis,
          const std::string &_config, const std::string &_target,
          const bool _preview, const bool _inverted = false);

      /// \brief Callback when an entity's scale has changed.
      /// \param[in] _name Name of entity.
      /// \param[in] _scale New scale.
      private: void OnEntityScaleChanged(const std::string &_name,
          const ignition::math::Vector3d &_scale);

      /// \brief Callback when an entity's pose has changed.
      /// \param[in] _name Name of entity.
      /// \param[in] _pose New pose.
      /// \param[in] _isFinal Whether this is the final pose or it is still
      /// being manipulated.
      private: void OnEntityMoved(const std::string &_name,
          const ignition::math::Pose3d &_pose, const bool _isFinal);

      /// \brief Deselect anything whose selection is handled here, such as
      /// links and model plugins.
      private: void DeselectAll();

      /// \brief Deselect all currently selected entities.
      private: void DeselectAllEntities();

      /// \brief Deselect all currently selected model plugins.
      private: void DeselectAllModelPlugins();

      /// \brief Callback when receiving a request to scale a link.
      /// \param[in] _name Link name.
      /// \param[in] _scales New scale for each child of the link.
      private: void OnRequestLinkScale(const std::string &_name,
          const std::map<std::string, ignition::math::Vector3d> &_scales);

      /// \brief Callback when receiving a request to move a link.
      /// \param[in] _name Link name.
      /// \param[in] _pose New link pose.
      private: void OnRequestLinkMove(const std::string &_name,
          const ignition::math::Pose3d &_pose);

      /// \brief Callback when receiving a request to move a nested model.
      /// \param[in] _name Nested model name.
      /// \param[in] _pose New nested model pose.
      private: void OnRequestNestedModelMove(const std::string &_name,
          const ignition::math::Pose3d &_pose);

      /// \brief Set visibilty of a visual recursively while storing their
      /// original values
      /// \param[in] _name Name of visual.
      /// \param[in] _visible True to set the visual to be visible.
      private: void SetModelVisible(const std::string &_name,
          const bool _visible);

      /// \brief Set visibilty of a visual recursively while storing their
      /// original values
      /// \param[in] _visual Pointer to the visual.
      /// \param[in] _visible True to set the visual to be visible.
      private: void SetModelVisible(const rendering::VisualPtr &_visual,
          const bool _visible);

      /// \brief Show a link's context menu
      /// \param[in] _link Name of link the context menu is associated with.
      private: void ShowContextMenu(const std::string &_link);

      /// \brief Show a model plugin's context menu
      /// \param[in] _name Name of model plugin.
      private: void ShowModelPluginContextMenu(const std::string &_name);

      /// \brief Helper function to emit nestedModelInserted events.
      /// \param[in] _vis Visual representing the nested mdoel.
      private: void EmitNestedModelInsertedEvent(
          const rendering::VisualPtr &_vis) const;

      /// \brief Qt callback when a delete signal has been emitted. This is
      /// currently triggered by the context menu via right click.
      private slots: void OnDelete();

      /// \brief Qt callback when a delete signal has been emitted.
      /// \param[in] _name Name of the entity to delete.
      private slots: void OnDelete(const std::string &_name);

      /// \brief Qt Callback to open link inspector
      private slots: void OnOpenInspector();

      /// \brief Qt Callback to open model plugin inspector.
      /// \param[in] _name Name of model plugin.
      private slots: void OnOpenModelPluginInspector(const QString &_name);

      /// \brief Qt Callback to remove model plugin.
      /// \param[in] _name Name of model plugin.
      private slots: void OnRemoveModelPlugin(const QString &_name);

      /// \brief Qt signal when the a link has been added.
      Q_SIGNALS: void LinkAdded();

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<ModelCreatorPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
