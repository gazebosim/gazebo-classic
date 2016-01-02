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

#ifndef _GAZEBO_GUI_EDITORVIEW_PRIVATE_HH_
#define _GAZEBO_GUI_EDITORVIEW_PRIVATE_HH_

#include <map>
#include <string>
#include <vector>

#include "gazebo/common/CommonTypes.hh"

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class DoorItem;
    class BuildingMaker;
    class EditorItem;
    class FloorItem;
    class GrabberHandle;
    class GridLines;
    class LevelInspectorDialog;
    class StairsItem;
    class WallSegmentItem;
    class WindowItem;

    /// \internal
    /// \brief Private data for EditorView
    class EditorViewPrivate
    {
      /// \brief Current draw mode
      public: int drawMode;

      /// \brief Indicate whether or not a drawing operation is taking place.
      public: bool drawInProgress;

      /// \brief Indicate whether or not the floorplan is visible.
      public: bool floorplanVisible;

      /// \brief Indicate whether or not the editor items are visible.
      public: bool elementsVisible;

      /// \brief A list of wall segment items in the scene.
      public: std::vector<WallSegmentItem *> wallSegmentList;

      /// \brief A list of window items in the scene.
      public: std::vector<WindowItem *> windowList;

      /// \brief A list of door items in the scene.
      public: std::vector<DoorItem *> doorList;

      /// \brief A list of staircase items in the scene.
      public: std::vector<StairsItem *> stairsList;

      /// \brief A list of floor items in the scene.
      public: std::vector<FloorItem *> floorList;

      /// \brief Mapping between 2D editor items to 3D visuals.
      public: std::map<EditorItem *, std::string> itemToVisualMap;

      /// \brief A list of gui editor events connected to this view.
      public: std::vector<event::ConnectionPtr> connections;

      /// \brief Editor item currently attached to the mouse during a drawing
      /// operation.
      public: QGraphicsItem *currentMouseItem;

      /// \brief Currently selected editor item.
      public: QGraphicsItem *currentSelectedItem;

      /// \brief Building maker manages the creation of 3D visuals
      public: BuildingMaker *buildingMaker;

      /// \brief Current building level associated to the view.
      public: int currentLevel;

      /// \brief A list of building levels in the scene.
      public: std::vector<Level *> levels;

      /// \brief A counter that holds the total number of levels in the building
      /// model.
      public: int levelCounter;

      /// \brief Default height for levels
      public: double levelDefaultHeight;

      /// \brief Qt action for opening a building level inspector.
      public: QAction *openLevelInspectorAct;

      /// \brief Qt action for adding a level to the building model.
      public: QAction *addLevelAct;

      /// \brief Qt action for deleting a level from the building model.
      public: QAction *deleteLevelAct;

      /// \brief Rotation in degrees when a mouse is pressed and dragged for
      /// rotating an item.
      public: double mousePressRotation;

      /// \brief Inspector for a building level.
      public: LevelInspectorDialog *levelInspector;

      /// \brief Grid lines drawn on the background of the editor.
      public: GridLines *gridLines;

      /// \brief Scale (zoom level) of the editor view.
      public: double viewScale;

      /// \brief Indicate whether or not the wall will snap to a grabber
      /// during a draw wall operation.
      public: bool snapToGrabber;

      /// \brief Existing grabber to snap towards.
      public: GrabberHandle *snapGrabberOther;

      /// \brief Currently held grabber which will be snapped.
      public: GrabberHandle *snapGrabberCurrent;

      /// \brief Text tooltip to follow the mouse.
      public: QGraphicsTextItem *mouseTooltip;
    };
  }
}

#endif
