/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_GUI_BUILDING_BUILDINGMAKERPRIVATE_HH_
#define _GAZEBO_GUI_BUILDING_BUILDINGMAKERPRIVATE_HH_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <sdf/sdf.hh>

#include "gazebo/common/CommonTypes.hh"

#include "gazebo/gui/qt.h"
#include "gazebo/gui/building/BuildingMaker.hh"

#include "gazebo/rendering/RenderTypes.hh"

#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  namespace gui
  {
    class BuildingModelManip;
    class SaveEntityDialog;

    /// \internal
    /// \brief Private data for BuildingMaker
    class BuildingMakerPrivate
    {
      /// \enum SaveState
      /// \brief Save states for the building editor.
      public: enum SaveState
      {
        // NEVER_SAVED: The building has never been saved.
        NEVER_SAVED,

        // ALL_SAVED: All changes have been saved.
        ALL_SAVED,

        // UNSAVED_CHANGES: Has been saved before, but has unsaved changes.
        UNSAVED_CHANGES
      };

      /// \brief A map of building part names to model manip objects which
      /// manage the visuals representing the building part.
      public: std::map<std::string, BuildingModelManip *> allItems;

      /// \brief A map of building part names to model manip objects which
      /// manage the visuals representing the building part.
      public: std::map<std::string, std::vector<std::string>> attachmentMap;

      /// \brief The building model in SDF format.
      public: sdf::SDFPtr modelSDF;

      /// \brief A template SDF of a simple box model.
      public: sdf::SDFPtr modelTemplateSDF;

      /// \brief Name of the building model.
      public: std::string modelName;

      /// \brief Folder name, which is the model name without spaces.
      public: std::string folderName;

      /// \brief Default name of building model
      public: std::string buildingDefaultName;

      /// \brief Name of the building model preview.
      public: std::string previewName;

      /// \brief The root visual of the building model preview.
      public: rendering::VisualPtr previewVisual;

      /// \brief Counter for the number of walls in the model.
      public: int wallCounter;

      /// \brief Counter for the number of windows in the model.
      public: int windowCounter;

      /// \brief Counter for the number of doors in the model.
      public: int doorCounter;

      /// \brief Counter for the number of staircases in the model.
      public: int stairsCounter;

      /// \brief Counter for the number of floors in the model.
      public: int floorCounter;

      /// \brief Store the current save state of the model.
      public: enum SaveState currentSaveState;

      /// \brief A list of gui editor events connected to the building maker.
      public: std::vector<event::ConnectionPtr> connections;

      /// \brief A dialog for setting building model name and save location.
      public: std::unique_ptr<SaveEntityDialog> saveDialog;

      /// \brief Visual that is currently hovered over by the mouse.
      public: rendering::VisualPtr hoverVis;

      /// \brief The color currently selected. If none is selected, it will be
      /// QColor::Invalid.
      public: QColor selectedColor;

      /// \brief The texture currently selected. If none is selected, it will be
      /// an empty string.
      public: QString selectedTexture;

      /// \brief The current level that is being edited.
      public: int currentLevel;

      /// \brief Node used to publish messages.
      public: transport::NodePtr node;

      /// \brief Publisher for factory messages.
      public: transport::PublisherPtr makerPub;
    };
  }
}

#endif
