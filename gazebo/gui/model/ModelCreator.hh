/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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
#ifndef _MODEL_CREATOR_HH_
#define _MODEL_CREATOR_HH_

#include <sdf/sdf.hh>

#include <list>
#include <map>
#include <string>
#include <vector>

#include "gazebo/common/KeyEvent.hh"
#include "gazebo/common/MouseEvent.hh"
#include "gazebo/math/Pose.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/gui/model/LinkInspector.hh"
#include "gazebo/gui/qt.h"

#include "gazebo/util/system.hh"

namespace boost
{
  class recursive_mutex;
}

namespace gazebo
{
  namespace msgs
  {
    class Visual;
  }

  namespace gui
  {
    class NestedModelData;
    class LinkData;
    class ModelPluginData;
    class SaveDialog;
    class JointMaker;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class ModelCreator ModelCreator.hh
    /// \brief Create and manage 3D visuals of a model with links and joints.
    class GZ_GUI_VISIBLE ModelCreator : public QObject
    {
      Q_OBJECT

      /// \enum Entity types
      /// \brief Unique identifiers for link types that can be created.
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
        // NEVER_SAVED: The model has never been saved.
        NEVER_SAVED,

        // ALL_SAVED: All changes have been saved.
        ALL_SAVED,

        // UNSAVED_CHANGES: Has been saved before, but has unsaved changes.
        UNSAVED_CHANGES
      };

      /// \brief Constructor
      public: ModelCreator();

      /// \brief Destructor
      public: virtual ~ModelCreator();

      /// \brief Set the name of the model.
      /// \param[in] _modelName Name of the model to set to.
      public: void SetModelName(const std::string &_modelName);

      /// \brief Get the name of the model.
      /// \return Name of model.
      public: std::string GetModelName() const;

      /// \brief Set save state upon a change to the model.
      public: void ModelChanged();

      /// \brief Callback for newing the model.
      private: void OnNew();

      /// \brief Helper function to manage writing files to disk.
      public: void SaveModelFiles();

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

      /// \brief Update callback on PreRender.
      private: void Update();

      /// \brief Finish the model and create the entity on the gzserver.
      public: void FinishModel();

      /// \brief Add a link to the model.
      /// \param[in] _type Type of link to add: box, cylinder, or sphere.
      /// \param[in] _size Size of the link.
      /// \param[in] _pose Pose of the link.
      /// \param[in] _samples Number of samples for polyline.
      /// \return Name of the link that has been added.
      public: std::string AddShape(EntityType _type,
          const math::Vector3 &_size = math::Vector3::One,
          const math::Pose &_pose = math::Pose::Zero,
          const std::string &_uri = "", unsigned int _samples = 5);

      /// \brief Add a box to the model.
      /// \param[in] _size Size of the box.
      /// \param[in] _pose Pose of the box.
      /// \return Name of the box that has been added.
      public: std::string AddBox(
          const math::Vector3 &_size = math::Vector3::One,
          const math::Pose &_pose = math::Pose::Zero);

      /// \brief Add a sphere to the model.
      /// \param[in] _radius Radius of the sphere.
      /// \param[in] _pose Pose of the sphere.
      /// \return Name of the sphere that has been added.
      public: std::string AddSphere(double _radius = 0.5,
          const math::Pose &_pose = math::Pose::Zero);

      /// \brief Add a cylinder to the model.
      /// \param[in] _radius Radius of the cylinder.
      /// \param[in] _length Length of the cylinder.
      /// \param[in] _pose Pose of the cylinder.
      /// \return Name of the cylinder that has been added.
      public: std::string AddCylinder(double _radius = 0.5,
          double _length = 1.0, const math::Pose &_pose = math::Pose::Zero);

      /// \brief Add a custom link to the model
      /// \param[in] _name Name of the custom link.
      /// \param[in] _scale Scale of the custom link.
      /// \param[in] _pose Pose of the custom link.
      /// \return Name of the custom that has been added.
      public: std::string AddCustom(const std::string &_name,
          const math::Vector3 &_scale = math::Vector3::One,
          const math::Pose &_pose = math::Pose::Zero);

      /// \brief Add a joint to the model.
      /// \param[in] _type Type of joint to add.
      public: void AddJoint(const std::string &_type);

      /// \brief Remove an entity from the model.
      /// \param[in] _entityName Name of the entity to remove
      public: void RemoveEntity(const std::string &_entityName);

      /// \brief Remove a model plugin from the model.
      /// \param[in] _pluginName Name of the model plugin to remove.
      public: void RemoveModelPlugin(const std::string &_pluginName);

      /// \brief Set the model to be static
      /// \param[in] _static True to make the model static.
      public: void SetStatic(bool _static);

      /// \brief Set the model to allow auto disable at rest.
      /// \param[in] _auto True to allow the model to auto disable.
      public: void SetAutoDisable(bool _auto);

      /// \brief Reset the model creator and the SDF.
      public: void Reset();

      /// \brief Stop the process of adding a link or joint to the model.
      public: void Stop();

      /// \brief Get joint maker
      /// \return Joint maker
      public: JointMaker *GetJointMaker() const;

      /// \brief Set the select state of a link.
      /// \param[in] _name Name of the link.
      /// \param[in] _selected True to select the link.
      public: void SetSelected(const std::string &_name, const bool selected);

      /// \brief Set the select state of a link visual.
      /// \param[in] _linkVis Pointer to the link visual.
      /// \param[in] _selected True to select the link.
      public: void SetSelected(rendering::VisualPtr _linkVis,
          const bool selected);

      /// \brief Get current save state.
      /// \return Current save state.
      public: enum SaveState GetCurrentSaveState() const;

      /// \brief Add an entity to the model
      /// \param[in] _sdf SDF describing the entity.
      public: void AddEntity(sdf::ElementPtr _sdf);

      /// \brief Add a link to the model
      /// \param[in] _type Type of link to be added
      public: void AddLink(EntityType _type);

      /// \brief Add a model plugin to the model
      /// \param[in] _pluginElem Pointer to plugin SDF element
      public: void AddModelPlugin(const sdf::ElementPtr _pluginElem);

      /// \brief Generate the SDF from model link and joint visuals.
      public: void GenerateSDF();

      /// \brief Helper function to generate link sdf from link data.
      /// \param[in] _link Link data used to generate the sdf.
      /// \return SDF element describing the link.
      private: sdf::ElementPtr GenerateLinkSDF(LinkData *_link);

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
      private slots: void OnEdit(bool _checked);

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

      /// \brief Callback when an entity is selected.
      /// \param[in] _name Name of entity.
      /// \param[in] _mode Select mode
      private: void OnSetSelectedEntity(const std::string &_name,
          const std::string &_mode);

      /// \brief Callback when a link is selected.
      /// \param[in] _name Name of link.
      /// \param[in] _selected True if the link is selected, false if
      /// deselected.
      private: void OnSetSelectedLink(const std::string &_name,
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
      private: void CreateLink(const rendering::VisualPtr &_visual);

      /// \brief Clone an existing link.
      /// \param[in] _linkName Name of link to be cloned.
      /// \return Cloned link.
      private: LinkData *CloneLink(const std::string &_linkName);

      /// \brief Create a link from an SDF with the specified parent visual.
      /// \param[in] _linkElem SDF element of the link that will be used to
      /// recreate its visual representation in the model editor.
      /// \param[in] _parentVis Parent visual that the link will be attached to.
      /// return Data describing this link.
      private: LinkData *CreateLinkFromSDF(const sdf::ElementPtr &_linkElem,
          const rendering::VisualPtr &_parentVis);

      /// \brief Open the link inspector.
      /// \param[in] _name Name of link.
      private: void OpenInspector(const std::string &_name);

      /// \brief Open the model plugin inspector.
      /// \param[in] _name Name of model plugin.
      private: void OpenModelPluginInspector(const std::string &_name);

      /// Spawn the entity in simulation.
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
      /// \return Data describing the model.
      private: NestedModelData *CreateModelFromSDF(const sdf::ElementPtr &_sdf,
          const rendering::VisualPtr &_parentVis = NULL);

      /// \brief Callback when a specific alignment configuration is set.
      /// \param[in] _axis Axis of alignment: x, y, or z.
      /// \param[in] _config Configuration: min, center, or max.
      /// \param[in] _target Target of alignment: first or last.
      /// \param[in] _bool True to preview alignment without publishing
      /// to server.
      private: void OnAlignMode(const std::string &_axis,
          const std::string &_config, const std::string &_target,
          bool _preview);

      /// \brief Callback when an entity's scale has changed.
      /// \param[in] _name Name of entity.
      /// \param[in] _scale New scale.
      private: void OnEntityScaleChanged(const std::string &_name,
          const math::Vector3 &_scale);

      /// \brief Deselect anything whose selection is handled here, such as
      /// links and model plugins.
      private: void DeselectAll();

      /// \brief Deselect all currently selected links.
      private: void DeselectAllLinks();

      /// \brief Deselect all currently selected model plugins.
      private: void DeselectAllModelPlugins();

      /// \brief Set visibilty of a visual recursively while storing their
      /// original values
      /// \param[in] _name Name of visual.
      /// \param[in] _visible True to set the visual to be visible.
      private: void SetModelVisible(const std::string &_name, bool _visible);

      /// \brief Set visibilty of a visual recursively while storing their
      /// original values
      /// \param[in] _visual Pointer to the visual.
      /// \param[in] _visible True to set the visual to be visible.
      private: void SetModelVisible(rendering::VisualPtr _visual,
          bool _visible);

      /// \brief Show a link's context menu
      /// \param[in] _link Name of link the context menu is associated with.
      private: void ShowContextMenu(const std::string &_link);

      /// \brief Show a model plugin's context menu
      /// \param[in] _name Name of model plugin.
      private: void ShowModelPluginContextMenu(const std::string &_name);

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

      /// \brief The model in SDF format.
      private: sdf::SDFPtr modelSDF;

      /// \brief A template SDF of a simple box model.
      private: sdf::SDFPtr modelTemplateSDF;

      /// \brief Name of the model.
      private: std::string modelName;

      /// \brief Folder name, which is the model name without spaces.
      private: std::string folderName;

      /// \brief Name of the model preview.
      private: static const std::string previewName;

      /// \brief The root visual of the model.
      private: rendering::VisualPtr previewVisual;

      /// \brief The root visual of the model.
      private: rendering::VisualPtr mouseVisual;

      /// \brief The pose of the model.
      private: ignition::math::Pose3d modelPose;

      /// \brief True to create a static model.
      private: bool isStatic;

      /// \brief True to auto disable model when it is at rest.
      private: bool autoDisable;

      /// \brief A list of gui editor events connected to the model creator.
      private: std::vector<event::ConnectionPtr> connections;

      /// \brief Counter for the number of links in the model.
      private: int linkCounter;

      /// \brief Counter for generating a unique model name.
      private: int modelCounter;

      /// \brief Type of entity being added.
      private: EntityType addEntityType;

      /// \brief A map of nested model names to and their visuals.
      private: std::map<std::string, NestedModelData *> allNestedModels;

      /// \brief A map of model link names to and their data.
      private: std::map<std::string, LinkData *> allLinks;

      /// \brief A map of model plugin names to and their data.
      private: std::map<std::string, ModelPluginData *> allModelPlugins;

      /// \brief A map of nested model link names and their visuals.
      private: std::map<std::string, LinkData *> nestedLinks;

      /// \brief Transport node
      private: transport::NodePtr node;

      /// \brief Publisher that publishes msg to the server once the model is
      /// created.
      private: transport::PublisherPtr makerPub;

      /// \brief Publisher that publishes delete entity msg to remove the
      /// editor visual.
      private: transport::PublisherPtr requestPub;

      /// \brief Joint maker.
      private: JointMaker *jointMaker;

      /// \brief origin of the model.
      private: math::Pose origin;

      /// \brief A list of selected link visuals.
      private: std::vector<rendering::VisualPtr> selectedLinks;

      /// \brief A list of selected model plugins.
      private: std::vector<std::string> selectedModelPlugins;

      /// \brief Names of links copied through g_copyAct
      private: std::vector<std::string> copiedLinkNames;

      /// \brief The last mouse event
      private: common::MouseEvent lastMouseEvent;

      /// \brief Qt action for opening the link inspector.
      private: QAction *inspectAct;

      /// \brief Name of link that is currently being inspected.
      private: std::string inspectName;

      /// \brief True if the model editor mode is active.
      private: bool active;

      /// \brief Current model manipulation mode.
      private: std::string manipMode;

      /// \brief Default name of the model.
      private: static const std::string modelDefaultName;

      /// \brief A dialog with options to save the model.
      private: SaveDialog *saveDialog;

      /// \brief Store the current save state of the model.
      private: enum SaveState currentSaveState;

      /// \brief Mutex to protect updates
      private: boost::recursive_mutex *updateMutex;

      /// \brief A list of link names whose scale has changed externally.
      private: std::map<std::string, math::Vector3> linkScaleUpdate;

      /// \brief Name of model on the server that is being edited here in the
      /// model editor.
      private: std::string serverModelName;

      /// \brief SDF element of the model on the server.
      private: sdf::ElementPtr serverModelSDF;

      /// \brief A map of all visuals of the model to be edited to their
      /// visibility.
      private: std::map<uint32_t, bool> serverModelVisible;

      /// \brief Name of the canonical model
      private: std::string canonicalModel;

      /// \brief Name of the canonical link in the model
      private: std::string canonicalLink;
    };
    /// \}
  }
}

#endif
