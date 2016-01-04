/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#ifndef _GAZEBO_GUI_MODELCREATOR_PRIVATE_HH_
#define _GAZEBO_GUI_MODELCREATOR_PRIVATE_HH_

#include <map>
#include <vector>
#include <string>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>

#include "gazebo/common/CommonTypes.hh"
#include "gazebo/common/MouseEvent.hh"

#include "gazebo/gui/qt.h"
#include "gazebo/gui/model/ModelCreator.hh"

#include "gazebo/rendering/RenderTypes.hh"

#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  namespace gui
  {
    class JointMaker;
    class LinkData;
    class ModelPluginData;
    class NestedModelData;
    class SaveDialog;

    /// \internal
    /// \class ModelCreator ModelCreator.hh
    /// \brief Private data for the ModelCreator class.
    class ModelCreatorPrivate
    {
      /// \brief Name of the model preview.
      public: static const std::string previewName;

      /// \brief Default name of the model.
      public: static const std::string modelDefaultName;

      /// \brief The model in SDF format.
      public: sdf::SDFPtr modelSDF;

      /// \brief A template SDF of a simple box model.
      public: sdf::SDFPtr modelTemplateSDF;

      /// \brief Name of the model.
      public: std::string modelName;

      /// \brief Folder name, which is the model name without spaces.
      public: std::string folderName;

      /// \brief The root visual of the model.
      public: rendering::VisualPtr previewVisual;

      /// \brief Visual currently being inserted into the model, which is
      /// attached to the mouse.
      public: rendering::VisualPtr mouseVisual;

      /// \brief The pose of the model.
      public: ignition::math::Pose3d modelPose;

      /// \brief True to create a static model.
      public: bool isStatic;

      /// \brief True to auto disable model when it is at rest.
      public: bool autoDisable;

      /// \brief A list of gui editor events connected to the model creator.
      public: std::vector<event::ConnectionPtr> connections;

      /// \brief Counter for the number of links in the model.
      public: int linkCounter;

      /// \brief Counter for generating a unique model name.
      public: int modelCounter;

      /// \brief Type of entity being added.
      public: ModelCreator::EntityType addEntityType;

      /// \brief A map of nested model names to and their visuals.
      public: std::map<std::string, NestedModelData *> allNestedModels;

      /// \brief A map of model link names to and their data.
      public: std::map<std::string, LinkData *> allLinks;

      /// \brief A map of model plugin names to and their data.
      public: std::map<std::string, ModelPluginData *> allModelPlugins;

      /// \brief Transport node
      public: transport::NodePtr node;

      /// \brief Publisher that publishes msg to the server once the model is
      /// created.
      public: transport::PublisherPtr makerPub;

      /// \brief Publisher that publishes delete entity msg to remove the
      /// editor visual.
      public: transport::PublisherPtr requestPub;

      /// \brief Joint maker.
      public: JointMaker *jointMaker;

      /// \brief origin of the model.
      public: ignition::math::Pose3d origin;

      /// \brief A list of selected link visuals.
      public: std::vector<rendering::VisualPtr> selectedLinks;

      /// \brief A list of selected model plugins.
      public: std::vector<std::string> selectedModelPlugins;

      /// \brief Names of links copied through g_copyAct
      public: std::vector<std::string> copiedLinkNames;

      /// \brief The last mouse event
      public: common::MouseEvent lastMouseEvent;

      /// \brief Qt action for opening the link inspector.
      public: QAction *inspectAct;

      /// \brief Name of link that is currently being inspected.
      public: std::string inspectName;

      /// \brief True if the model editor mode is active.
      public: bool active;

      /// \brief Current model manipulation mode.
      public: std::string manipMode;

      /// \brief A dialog with options to save the model.
      public: SaveDialog *saveDialog;

      /// \brief Store the current save state of the model.
      public: ModelCreator::SaveState currentSaveState;

      /// \brief Mutex to protect updates
      public: std::recursive_mutex updateMutex;

      /// \brief A list of link names whose scale has changed externally.
      public: std::map<std::string, ignition::math::Vector3d> linkScaleUpdate;

      /// \brief Name of model on the server that is being edited here in the
      /// model editor.
      public: std::string serverModelName;

      /// \brief SDF element of the model on the server.
      public: sdf::ElementPtr serverModelSDF;

      /// \brief A map of all visuals of the model to be edited to their
      /// visibility.
      public: std::map<uint32_t, bool> serverModelVisible;

      /// \brief Name of the canonical model
      public: std::string canonicalModel;

      /// \brief Name of the canonical link in the model
      public: std::string canonicalLink;
    };
  }
}
#endif
