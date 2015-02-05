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
    class PartData;
    class SaveDialog;
    class JointMaker;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class ModelCreator ModelCreator.hh
    /// \brief Create and manage 3D visuals of a model with parts and joints.
    class GAZEBO_VISIBLE ModelCreator : public QObject
    {
      Q_OBJECT

      /// \enum Joint types
      /// \brief Unique identifiers for joint types that can be created.
      public: enum PartType
      {
        /// \brief none
        PART_NONE,
        /// \brief Box
        PART_BOX,
        /// \brief Sphere
        PART_SPHERE,
        /// \brief Cylinder
        PART_CYLINDER,
        /// \brief Custom
        PART_CUSTOM
      };

      /// \enum SaveState
      /// \brief Save states for the model editor.
      private: enum SaveState
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
      private: void SaveModelFiles();

      /// \brief Callback for saving the model.
      /// \return True if the user chose to save, false if the user cancelled.
      private: bool OnSave();

      /// \brief Callback for selecting a folder and saving the model.
      /// \return True if the user chose to save, false if the user cancelled.
      private: bool OnSaveAs();

      /// \brief Callback for when the name is changed through the Palette.
      /// \param[in] _modelName The newly entered model name.
      private: void OnNameChanged(const std::string &_modelName);

      /// \brief Callback received when exiting the editor mode.
      private: void OnExit();

      /// \brief Update callback on PreRender.
      private: void Update();

      /// \brief Finish the model and create the entity on the gzserver.
      public: void FinishModel();

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

      /// \brief Add a custom part to the model
      /// \param[in] _name Name of the custom part.
      /// \param[in] _scale Scale of the custom part.
      /// \param[in] _pose Pose of the custom part.
      /// \return Name of the custom that has been added.
      public: std::string AddCustom(const std::string &_name,
          const math::Vector3 &_scale = math::Vector3::One,
          const math::Pose &_pose = math::Pose::Zero);

      /// \brief Add a joint to the model.
      /// \param[in] _type Type of joint to add.
      public: void AddJoint(const std::string &_type);

      /// \brief Remove a part from the model.
      /// \param[in] _partName Name of the part to remove
      public: void RemovePart(const std::string &_partName);

      /// \brief Set the model to be static
      /// \param[in] _static True to make the model static.
      public: void SetStatic(bool _static);

      /// \brief Set the model to allow auto disable at rest.
      /// \param[in] _auto True to allow the model to auto disable.
      public: void SetAutoDisable(bool _auto);

      /// \brief Reset the model creator and the SDF.
      public: void Reset();

      /// \brief Stop the process of adding a part or joint to the model.
      public: void Stop();

      /// \brief Get joint maker
      /// \return Joint maker
      public: JointMaker *GetJointMaker() const;

      /// \brief Add a part to the model
      /// \param[in] _type Type of part to be added
      public: void AddPart(PartType _type);

      /// \brief Generate the SDF from model part and joint visuals.
      public: void GenerateSDF();

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
      /// \param[in] _mode Select model
      private: void OnSetSelectedEntity(const std::string &_name,
          const std::string &_mode);

      /// \brief Create part with default properties from a visual. This
      /// function creates a link that will become the parent of the
      /// input visual. A collision visual with the same geometry as the input
      /// visual will also be added to the link.
      /// \param[in] _visual Visual used to create the part.
      private: void CreatePart(const rendering::VisualPtr &_visual);

      /// \brief Clone an existing part.
      /// \param[in] _partName Name of part to be cloned.
      /// \return Cloned part.
      private: PartData *ClonePart(const std::string &_partName);

      /// \brief TODO
      private: void CreatePartFromSDF(sdf::ElementPtr _linkElem);

      /// \brief Open the part inspector.
      /// \param[in] _name Name of part.
      private: void OpenInspector(const std::string &_name);

      // Documentation inherited
      private: virtual void CreateTheEntity();

      /// \brief Internal init function.
      private: bool Init();

      /// \brief Create an empty model.
      /// \return Name of the model created.
      private: std::string CreateModel();

      /// \brief TODO
      private: void LoadSDF(sdf::ElementPtr _sdf);
      /// \brief Callback when a specific alignment configuration is set.
      /// \param[in] _axis Axis of alignment: x, y, or z.
      /// \param[in] _config Configuration: min, center, or max.
      /// \param[in] _target Target of alignment: first or last.
      /// \param[in] _bool True to preview alignment without publishing
      /// to server.
      private: void OnAlignMode(const std::string &_axis,
          const std::string &_config, const std::string &_target,
          bool _preview);

      /// \brief Deselect all currently selected visuals.
      private: void DeselectAll();

      /// \brief Qt callback when a delete signal has been emitted.
      /// \param[in] _name Name of the entity to delete.
      private slots: void OnDelete(const std::string &_name="");

      /// \brief Qt Callback to open part inspector
      private slots: void OnOpenInspector();

      /// \brief Qt signal when the a part has been added.
      Q_SIGNALS: void PartAdded();

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
      private: math::Pose modelPose;

      /// \brief True to create a static model.
      private: bool isStatic;

      /// \brief True to auto disable model when it is at rest.
      private: bool autoDisable;

      /// \brief A list of gui editor events connected to the model creator.
      private: std::vector<event::ConnectionPtr> connections;

      /// \brief Counter for the number of parts in the model.
      private: int partCounter;

      /// \brief Counter for generating a unique model name.
      private: int modelCounter;

      /// \brief Type of part being added.
      private: PartType addPartType;

      /// \brief A map of model part names to and their visuals.
      private: std::map<std::string, PartData *> allParts;

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

      /// \brief A list of selected visuals.
      private: std::vector<rendering::VisualPtr> selectedVisuals;

      /// \brief Names of parts copied through g_copyAct
      private: std::vector<std::string> copiedPartNames;

      /// \brief The last mouse event
      private: common::MouseEvent lastMouseEvent;

      /// \brief Qt action for opening the part inspector.
      private: QAction *inspectAct;

      /// \brief Part visual that is currently being inspected.
      private: rendering::VisualPtr inspectVis;

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

      /// \brief A list of model sdfs to load in the model editor.
      private: std::vector<sdf::ElementPtr> sdfToLoad;

      /// \brief Mutex to protect allParts
      private: boost::recursive_mutex *updateMutex;
    };
    /// \}
  }
}

#endif
