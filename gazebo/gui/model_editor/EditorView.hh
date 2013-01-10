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

#ifndef _EDITOR_VIEW_HH_
#define _EDITOR_VIEW_HH_

#include "gazebo/gui/qt.h"
#include "gazebo/common/Event.hh"

namespace gazebo
{
  namespace gui
  {
    class EditorItem;

    class WindowItem;

    class StairsItem;

    class DoorItem;

    class WallItem;

    class FloorItem;

    class BuildingMaker;

    class LevelInspectorDialog;

    class GridLines;

    class Level
    {
      public: int level;

      public: std::string name;

      public: double height;
    };

    class EditorView : public QGraphicsView
    {
      Q_OBJECT

      public: EditorView(QWidget *_parent = 0);

      public: ~EditorView();

      public: enum modelTypes{NONE, WALL, WINDOW, DOOR, STAIRS};

      public: enum mouseActions{Select, Translate, Rotate};

      public: void CreateItem3D(EditorItem *item);

      public: void DeleteItem(EditorItem *_item);

      private: void resizeEvent(QResizeEvent *_event);

      private: void scrollContentsBy (int _dx, int _dy);

      private: void contextMenuEvent(QContextMenuEvent *event);

      private: void wheelEvent(QWheelEvent *_event);

      private: void mousePressEvent(QMouseEvent *_event);

      private: void mouseReleaseEvent(QMouseEvent *_event);

      private: void mouseMoveEvent(QMouseEvent *_event);

      private: void mouseDoubleClickEvent(QMouseEvent *_event);

      private: void keyPressEvent(QKeyEvent *_event);

      private: void DrawWall(const QPoint &_pos);

      private: void DrawWindow(const QPoint &_pos);

      private: void DrawDoor(const QPoint &_pos);

      private: void DrawStairs(const QPoint &_pos);

      private: void OnCreateEditorItem(const std::string &_type);

      private: void OnSaveModel(const std::string &_modelName,
          const std::string &_savePath);

      private: void OnFinishModel();

      private: void OnDiscardModel();

      private slots: void OnAddLevel();

      private slots: void OnDeleteLevel();

      private slots: void OnLevelApply();

      private: void DeleteLevel(int _level);

      private: void OnChangeLevel(int _level);

      private slots: void OnOpenLevelInspector();

      private: void CancelDrawMode();

      private: int drawMode;

      private: int mouseMode;

      private: bool drawInProgress;

      private: std::vector<WallItem*> wallList;

      private: std::vector<WindowItem*> windowList;

      private: std::vector<DoorItem*> doorList;

      private: std::vector<StairsItem*> stairsList;

      private: std::vector<FloorItem*> floorList;

      private: std::map<EditorItem *, std::string> itemToModelMap;

      private: std::vector<event::ConnectionPtr> connections;

      private: QGraphicsItem *currentMouseItem;

      private: QGraphicsItem *currentSelectedItem;

      private: BuildingMaker *buildingMaker;

      private: std::string lastWallSegmentName;

      private: int currentLevel;

      private: std::vector<Level *> levels;

      private: int levelCounter;

      private: QAction *openLevelInspectorAct;

      private: QAction *addLevelAct;

      private: QAction *deleteLevelAct;

      private: double grabberDragRotation;

      private: LevelInspectorDialog *levelInspector;

      private: GridLines *gridLines;

      private: double viewScale;
    };
  }
}

#endif
