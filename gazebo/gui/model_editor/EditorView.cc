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

#include "gui/model_editor/BuildingItem.hh"
#include "gui/model_editor/GridLines.hh"
#include "gui/model_editor/EditorItem.hh"
#include "gui/model_editor/RectItem.hh"
#include "gui/model_editor/WindowItem.hh"
#include "gui/model_editor/DoorItem.hh"
#include "gui/model_editor/StairsItem.hh"
#include "gui/model_editor/LineSegmentItem.hh"
#include "gui/model_editor/PolylineItem.hh"
#include "gui/model_editor/WallItem.hh"
#include "gui/model_editor/BuildingMaker.hh"
#include "gui/model_editor/LevelInspectorDialog.hh"
#include "gui/model_editor/EditorEvents.hh"
#include "gui/model_editor/EditorView.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
EditorView::EditorView(QWidget *_parent)
  : QGraphicsView(_parent), currentMouseItem(0)
{
  this->setObjectName("editorView");

  this->drawMode = NONE;
  this->drawInProgress = false;
  this->mouseMode = Select;

  this->connections.push_back(
  gui::Events::ConnectCreateEditorItem(
    boost::bind(&EditorView::OnCreateEditorItem, this, _1)));

  this->connections.push_back(
  gui::Events::ConnectFinishModel(
    boost::bind(&EditorView::OnFinishModel, this, _1, _2)));

  this->connections.push_back(
  gui::Events::ConnectAddLevel(
    boost::bind(&EditorView::OnAddLevel, this, _1, _2)));

  this->connections.push_back(
  gui::Events::ConnectChangeLevel(
    boost::bind(&EditorView::OnChangeLevel, this, _1)));

  buildingMaker = new BuildingMaker();
  this->currentLevel = 0;

  this->levelNames[0] = "Level 1";

  this->openLevelInspectorAct = new QAction(tr("&Open Level Inspector"), this);
  this->openLevelInspectorAct->setStatusTip(tr("Open Level Inspector"));
  connect(this->openLevelInspectorAct, SIGNAL(triggered()),
    this, SLOT(OnOpenLevelInspector()));
}

/////////////////////////////////////////////////
EditorView::~EditorView()
{
  if (buildingMaker)
    delete buildingMaker;
}

/////////////////////////////////////////////////
void EditorView::contextMenuEvent(QContextMenuEvent *_event)
{
  QMenu menu(this);
  menu.addAction(this->openLevelInspectorAct);
  menu.exec(_event->globalPos());
}


/////////////////////////////////////////////////
void EditorView::mousePressEvent(QMouseEvent *_event)
{
  QGraphicsView::mousePressEvent(_event);
}

/////////////////////////////////////////////////
void EditorView::mouseReleaseEvent(QMouseEvent *_event)
{
  switch (drawMode)
  {
    case NONE:
      break;
    case WALL:
      this->DrawLine(_event->pos());
      break;
    case WINDOW:
      if (drawInProgress)
      {
        this->windowList.push_back(dynamic_cast<WindowItem*>(
            this->currentMouseItem));
        this->drawMode = NONE;
        this->drawInProgress = false;
      }

      break;
    case DOOR:
      if (drawInProgress)
      {
        this->doorList.push_back(dynamic_cast<DoorItem*>(
            this->currentMouseItem));
        this->drawMode = NONE;
        this->drawInProgress = false;
      }
      break;
    case STAIRS:
      if (drawInProgress)
      {
        this->stairsList.push_back(dynamic_cast<StairsItem*>(
            this->currentMouseItem));
        this->drawMode = NONE;
        this->drawInProgress = false;
      }
    default:
      break;
  }
  if (!drawInProgress)
  {
    QGraphicsView::mouseReleaseEvent(_event);
  }
}

/////////////////////////////////////////////////
void EditorView::mouseMoveEvent(QMouseEvent *_event)
{
  switch (drawMode)
  {
    case NONE:
      break;
    case WALL:
    {
      WallItem *wallItem = dynamic_cast<WallItem*>(this->currentMouseItem);
      if (this->drawInProgress && wallItem)
      {
        if (wallItem)
        {
          LineSegmentItem *segment = wallItem->GetSegment(
              wallItem->GetSegmentCount()-1);
          QPointF p1 = segment->mapToScene(segment->line().p1());
          QPointF p2 = this->mapToScene(_event->pos());
          QLineF line(p1, p2);
          double angle = line.angle();
          double range = 10;
          if ((angle < range) || (angle > (360 - range)) ||
              ((angle > (180 - range)) && (angle < (180 + range))))
          {
            wallItem->SetVertexPosition(wallItem->GetVertexCount()-1,
                QPointF(p2.x(), p1.y()));
          }
          else if (((angle > (90 - range)) && (angle < (90 + range))) ||
              ((angle > (270 - range)) && (angle < (270 + range))))
          {
            wallItem->SetVertexPosition(wallItem->GetVertexCount()-1,
                QPointF(p1.x(), p2.y()));
          }
          else
          {
            wallItem->SetVertexPosition(wallItem->GetVertexCount()-1, p2);
          }
        }
      }
      break;
    }
    case WINDOW:
      this->DrawWindow(_event->pos());
      break;
    case DOOR:
      this->DrawDoor(_event->pos());
      break;
    case STAIRS:
      this->DrawStairs(_event->pos());
      break;
    default:
      break;
  }
  QGraphicsView::mouseMoveEvent(_event);
}

/////////////////////////////////////////////////
void EditorView::keyPressEvent(QKeyEvent *_event)
{
  if (_event->key() == Qt::Key_Delete)
    {
    QList<QGraphicsItem *> selectedItems = this->scene()->selectedItems();

    for (int i = 0; i < selectedItems.size(); ++i)
    {
      QGraphicsItem *item = selectedItems[i];
      if (dynamic_cast<WallItem *>(item))
        wallList.remove(dynamic_cast<WallItem *>(item));
      else if (dynamic_cast<WindowItem *>(item))
          windowList.remove(dynamic_cast<WindowItem *>(item));
      else if (dynamic_cast<DoorItem *>(item))
        doorList.remove(dynamic_cast<DoorItem *>(item));
      else if (dynamic_cast<StairsItem *>(item))
        stairsList.remove(dynamic_cast<StairsItem *>(item));
      delete selectedItems[i];
    }
    drawMode = NONE;
    this->drawInProgress = false;
    this->currentMouseItem = NULL;
    QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
  }
}

/////////////////////////////////////////////////
void EditorView::mouseDoubleClickEvent(QMouseEvent *_event)
{
  if (this->drawMode == WALL)
  {
    WallItem* wallItem = dynamic_cast<WallItem*>(this->currentMouseItem);
    wallItem->PopEndPoint();
    wallList.push_back(wallItem);
//    this->buildingMaker->RemoveWall(this->lastWallSegmentName);
    this->lastWallSegmentName = "";
    this->drawMode = NONE;
    this->drawInProgress = false;
  }
  QGraphicsView::mouseDoubleClickEvent(_event);
}

/////////////////////////////////////////////////
void EditorView::DrawLine(QPoint _pos)
{
  WallItem *wallItem = NULL;
  if (!drawInProgress)
  {
    QPointF pointStart = mapToScene(_pos);
    QPointF pointEnd = pointStart;

    wallItem = new WallItem(pointStart, pointEnd);
    wallItem->SetLevel(this->currentLevel);
    wallItem->SetLevelBaseHeight(this->levelHeights[this->currentLevel]);
    this->scene()->addItem(wallItem);
    this->currentMouseItem = wallItem;
    this->drawInProgress = true;

    lastLineCornerPos = _pos;
  }
  else
  {
    wallItem = dynamic_cast<WallItem*>(this->currentMouseItem);
    if (wallItem)
    {
      LineSegmentItem *segment = wallItem->GetSegment(
          wallItem->GetSegmentCount()-1);

      wallItem->AddPoint(segment->mapToScene(segment->line().p2()));
    }
  }

  if (wallItem)
  {
    LineSegmentItem *segment = wallItem->GetSegment(
        wallItem->GetSegmentCount()-1);

    QVector3D segmentPosition = segment->GetScenePosition();
    segmentPosition.setZ(wallItem->GetLevelBaseHeight() + segmentPosition.z());
    std::string wallSegmentName = this->buildingMaker->AddWall(
        segment->GetSize(), segmentPosition, segment->GetSceneRotation());
    this->lastWallSegmentName = wallSegmentName;
    this->buildingMaker->ConnectItem(wallSegmentName, segment);
    this->buildingMaker->ConnectItem(wallSegmentName, wallItem);
  }
}

/////////////////////////////////////////////////
void EditorView::DrawWindow(QPoint _pos)
{
  WindowItem *windowItem = NULL;
  if (!drawInProgress)
  {
    windowItem = new WindowItem();
    windowItem->SetLevel(this->currentLevel);
    windowItem->SetLevelBaseHeight(this->levelHeights[this->currentLevel]);
    this->scene()->addItem(windowItem);
    this->currentMouseItem = windowItem;

    QVector3D windowPosition = windowItem->GetScenePosition();
    windowPosition.setZ(windowItem->GetLevelBaseHeight() + windowPosition.z());
    std::string windowName = this->buildingMaker->AddWindow(
        windowItem->GetSize(), windowPosition, windowItem->GetSceneRotation());
    this->buildingMaker->ConnectItem(windowName, windowItem);

    this->drawInProgress = true;
  }
  windowItem = dynamic_cast<WindowItem*>(this->currentMouseItem);
  if (windowItem)
  {
    QPointF scenePos = this->mapToScene(_pos);
    windowItem->SetPosition(scenePos.x(), scenePos.y());
  }
}

/////////////////////////////////////////////////
void EditorView::DrawDoor(QPoint _pos)
{
  DoorItem *doorItem = NULL;
  if (!drawInProgress)
  {
    doorItem = new DoorItem();
    doorItem->SetLevel(this->currentLevel);
    doorItem->SetLevelBaseHeight(this->levelHeights[this->currentLevel]);
    this->scene()->addItem(doorItem);
    this->currentMouseItem = doorItem;
    QVector3D doorPosition = doorItem->GetScenePosition();
    doorPosition.setZ(doorItem->GetLevelBaseHeight() + doorPosition.z());
    std::string doorName = this->buildingMaker->AddDoor(
        doorItem->GetSize(), doorPosition, doorItem->GetSceneRotation());
    this->buildingMaker->ConnectItem(doorName, doorItem);

    this->drawInProgress = true;
  }
  doorItem = dynamic_cast<DoorItem*>(currentMouseItem);
  if (doorItem)
  {
    QPointF scenePos = this->mapToScene(_pos);
    doorItem->SetPosition(scenePos.x(), scenePos.y());
  }
}

/////////////////////////////////////////////////
void EditorView::DrawStairs(QPoint _pos)
{
  StairsItem *stairsItem = NULL;
  if (!drawInProgress)
  {
    stairsItem = new StairsItem();
    stairsItem->SetLevel(this->currentLevel);
    stairsItem->SetLevelBaseHeight(this->levelHeights[this->currentLevel]);
    this->scene()->addItem(stairsItem);
    this->currentMouseItem = stairsItem;

    QVector3D stairsPosition = stairsItem->GetScenePosition();
    stairsPosition.setZ(stairsItem->GetLevelBaseHeight() + stairsPosition.z());
    std::string stairsName = this->buildingMaker->AddStairs(
        stairsItem->GetSize(), stairsPosition, stairsItem->GetSceneRotation(),
        stairsItem->GetSteps());

    this->buildingMaker->ConnectItem(stairsName, stairsItem);

    this->drawInProgress = true;
  }
  stairsItem = dynamic_cast<StairsItem*>(this->currentMouseItem);
  if (stairsItem)
  {
    QPointF scenePos = this->mapToScene(_pos);
    stairsItem->SetPosition(scenePos.x(), scenePos.y());
  }
}

/////////////////////////////////////////////////
void EditorView::CloneItem3D(EditorItem* _item)
{
  if (_item->GetType() == "Stairs")
  {
    StairsItem *stairsItem = dynamic_cast<StairsItem *>(_item);
    QVector3D stairsPosition = stairsItem->GetScenePosition();
    stairsPosition.setZ(stairsItem->GetLevelBaseHeight() + stairsPosition.z());
    std::string stairsName = this->buildingMaker->AddStairs(
        stairsItem->GetSize(), stairsPosition, stairsItem->GetSceneRotation(),
        stairsItem->GetSteps());
    this->buildingMaker->ConnectItem(stairsName, stairsItem);
  }
}

/////////////////////////////////////////////////
void EditorView::OnCreateEditorItem(const std::string &_type)
{

//qDebug() << " create editor itme " << _type.c_str();

  if (_type == "Wall")
    this->drawMode = WALL;
  else if (_type == "Window")
    this->drawMode = WINDOW;
  else if (_type == "Door")
    this->drawMode = DOOR;
  else if (_type == "Stairs")
    this->drawMode = STAIRS;

  if (this->drawInProgress && this->currentMouseItem)
  {
    this->scene()->removeItem(this->currentMouseItem);
    this->currentMouseItem = NULL;
    this->drawInProgress = false;
  }
}

/////////////////////////////////////////////////
void EditorView::OnFinishModel(const std::string &_modelName,
    const std::string &_savePath)
{
  if (this->buildingMaker)
  {
    this->buildingMaker->SetModelName(_modelName);
    this->buildingMaker->FinishModel();
    this->buildingMaker->SaveToSDF(_savePath);
  }
}

/////////////////////////////////////////////////
void EditorView::OnAddLevel(int _newLevel, std::string _levelName)
{
  this->currentLevel = _newLevel;
  this->levelNames[_newLevel] = _levelName;

  if (wallList.size() == 0)
  {
    this->levelHeights[_newLevel] = 0;
    return;
  }

  std::list<WallItem *>::iterator wallIt = this->wallList.begin();
  double maxHeight = (*wallIt)->GetHeight();
  double wallHeight = (*wallIt)->GetHeight() + (*wallIt)->GetLevelBaseHeight();
  int wallLevel = 0;


  wallIt++;
  for (wallIt; wallIt != this->wallList.end(); ++wallIt)
  {
    wallHeight = (*wallIt)->GetHeight() + (*wallIt)->GetLevelBaseHeight();
    if ( wallHeight > maxHeight)
    {
      maxHeight = wallHeight;
      wallLevel = (*wallIt)->GetLevel();
    }
  }

  this->levelHeights[_newLevel] = maxHeight;

  std::vector<WallItem *> newWalls;
  for (std::list<WallItem *>::iterator it = wallList.begin();
      it  != this->wallList.end(); ++it)
  {
    if ((*it)->GetLevel() != wallLevel)
      continue;

    WallItem *wallItem = (*it)->Clone();
    wallItem->SetLevel(_newLevel);
    wallItem->SetLevelBaseHeight(this->levelHeights[this->currentLevel]);
    this->scene()->addItem(wallItem);
    newWalls.push_back(wallItem);
    for (unsigned int j = 0; j < wallItem->GetSegmentCount(); ++j)
    {
      LineSegmentItem *segment = wallItem->GetSegment(j);
      QVector3D segmentSize = segment->GetSize();
      segmentSize.setZ(wallItem->GetHeight());
      QVector3D segmentPosition = segment->GetScenePosition();
      segmentPosition.setZ(wallItem->GetLevelBaseHeight() + segmentPosition.z());
      std::string wallSegmentName = this->buildingMaker->AddWall(
          segmentSize, segmentPosition, segment->GetSceneRotation());

      this->buildingMaker->ConnectItem(wallSegmentName, segment);
      this->buildingMaker->ConnectItem(wallSegmentName, wallItem);
    }
  }
  this->wallList.insert(this->wallList.end(), newWalls.begin(),
      newWalls.end());
}

/////////////////////////////////////////////////
void EditorView::OnChangeLevel(int _level)
{
  this->currentLevel = _level;
  for (std::list<WallItem *>::iterator it = this->wallList.begin();
      it != this->wallList.end(); ++it)
  {
    if ((*it)->GetLevel() != _level)
      (*it)->setVisible(false);
    else (*it)->setVisible(true);
  }
  for (std::list<WindowItem *>::iterator it = this->windowList.begin();
      it != this->windowList.end(); ++it)
  {
    if ((*it)->GetLevel() != _level)
      (*it)->setVisible(false);
    else (*it)->setVisible(true);
  }
  for (std::list<DoorItem *>::iterator it = this->doorList.begin();
      it != this->doorList.end(); ++it)
  {
    if ((*it)->GetLevel() != _level)
      (*it)->setVisible(false);
    else (*it)->setVisible(true);
  }
  for (std::list<StairsItem *>::iterator it = this->stairsList.begin();
      it != this->stairsList.end(); ++it)
  {
    if ((*it)->GetLevel() != _level)
      (*it)->setVisible(false);
    else (*it)->setVisible(true);
  }
}

/////////////////////////////////////////////////
void EditorView::OnOpenLevelInspector()
{
  LevelInspectorDialog dialog;
  dialog.SetLevelName(this->levelNames[this->currentLevel]);
  if (dialog.exec() == QDialog::Accepted)
  {
    std::string newLevelName = dialog.GetLevelName();
    this->levelNames[this->currentLevel] = newLevelName;
    emit gui::Events::changeLevelName(this->currentLevel, newLevelName);
  }
}
