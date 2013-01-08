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

#include "gazebo/gui/model_editor/BuildingItem.hh"
#include "gazebo/gui/model_editor/GridLines.hh"
#include "gazebo/gui/model_editor/EditorItem.hh"
#include "gazebo/gui/model_editor/RectItem.hh"
#include "gazebo/gui/model_editor/WindowItem.hh"
#include "gazebo/gui/model_editor/DoorItem.hh"
#include "gazebo/gui/model_editor/StairsItem.hh"
#include "gazebo/gui/model_editor/FloorItem.hh"
#include "gazebo/gui/model_editor/LineSegmentItem.hh"
#include "gazebo/gui/model_editor/PolylineItem.hh"
#include "gazebo/gui/model_editor/WallItem.hh"
#include "gazebo/gui/model_editor/BuildingMaker.hh"
#include "gazebo/gui/model_editor/LevelInspectorDialog.hh"
#include "gazebo/gui/model_editor/EditorEvents.hh"
#include "gazebo/gui/model_editor/EditorView.hh"

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
  gui::editor::Events::ConnectCreateEditorItem(
    boost::bind(&EditorView::OnCreateEditorItem, this, _1)));

  this->connections.push_back(
  gui::editor::Events::ConnectSaveModel(
    boost::bind(&EditorView::OnSaveModel, this, _1, _2)));

  this->connections.push_back(
  gui::editor::Events::ConnectFinishModel(
    boost::bind(&EditorView::OnFinishModel, this)));

  this->connections.push_back(
  gui::editor::Events::ConnectDiscardModel(
    boost::bind(&EditorView::OnDiscardModel, this)));

  this->connections.push_back(
  gui::editor::Events::ConnectAddLevel(
    boost::bind(&EditorView::OnAddLevel, this)));

  this->connections.push_back(
  gui::editor::Events::ConnectChangeLevel(
    boost::bind(&EditorView::OnChangeLevel, this, _1)));

  this->grabberDragRotation = 0;

  buildingMaker = new BuildingMaker();
  this->currentLevel = 0;

  this->levelHeights[0] = 0;
  this->levelNames[0] = "Level 1";

  this->levelInspector = new LevelInspectorDialog();
  this->levelInspector->setModal(false);
  connect(this->levelInspector, SIGNAL(Applied()), this, SLOT(OnLevelApply()));

  this->openLevelInspectorAct = new QAction(tr("&Open Level Inspector"), this);
  this->openLevelInspectorAct->setStatusTip(tr("Open Level Inspector"));
  connect(this->openLevelInspectorAct, SIGNAL(triggered()),
    this, SLOT(OnOpenLevelInspector()));

  this->addLevelAct = new QAction(tr("&Add Level"), this);
  this->addLevelAct->setStatusTip(tr("Add Level"));
  connect(this->addLevelAct, SIGNAL(triggered()),
    this, SLOT(OnAddLevel()));
}

/////////////////////////////////////////////////
EditorView::~EditorView()
{
  if (this->buildingMaker)
    delete this->buildingMaker;

  delete this->levelInspector;
}

/////////////////////////////////////////////////
void EditorView::contextMenuEvent(QContextMenuEvent *_event)
{
  if (this->drawInProgress)
  {
    this->CancelDrawMode();
    _event->accept();
    return;
  }

  QGraphicsItem *item = this->scene()->itemAt(this->mapToScene(_event->pos()));
  if (item)
  {
    _event->ignore();
    QGraphicsView::contextMenuEvent(_event);
    return;
  }

  QMenu menu(this);
  menu.addAction(this->addLevelAct);
  menu.addAction(this->openLevelInspectorAct);
  menu.exec(_event->globalPos());
  _event->accept();
}

/////////////////////////////////////////////////
void EditorView::mousePressEvent(QMouseEvent *_event)
{
  if (!this->drawInProgress && this->drawMode != WALL
      && (_event->button() != Qt::RightButton))
  {
    QGraphicsItem *mouseItem =
        this->scene()->itemAt(this->mapToScene(_event->pos()));
    if (mouseItem && !mouseItem->isSelected())
    {
      EditorItem *editorItem = dynamic_cast<EditorItem*>(mouseItem);
      if (editorItem)
      {
        this->scene()->clearSelection();
        mouseItem->setSelected(true);
      }
    }
    QGraphicsView::mousePressEvent(_event);
  }
}

/////////////////////////////////////////////////
void EditorView::mouseReleaseEvent(QMouseEvent *_event)
{
  switch (drawMode)
  {
    case NONE:
      break;
    case WALL:
      this->DrawWall(_event->pos());
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
        if ((this->currentLevel) < static_cast<int>(floorList.size()))
        {
          EditorItem *item = dynamic_cast<EditorItem*>(this->currentMouseItem);
          this->buildingMaker->AttachObject(this->itemToModelMap[item],
              this->itemToModelMap[floorList[this->currentLevel]]);
        }
        this->drawMode = NONE;
        this->drawInProgress = false;

      }
    default:
      break;
  }

  if (!this->drawInProgress)
    this->currentMouseItem = NULL;

  QGraphicsView::mouseReleaseEvent(_event);
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
          // snap walls to 0/90/180 degrees
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


  // auto attach windows and doors to walls
  QGraphicsItem *grabber = this->scene()->mouseGrabberItem();
  if (!grabber)
    grabber = this->currentMouseItem;
  RectItem *editorItem = dynamic_cast<RectItem *>(grabber);
  if (grabber && editorItem && (editorItem->GetType() == "Window"
      || editorItem->GetType() == "Door") )
  {
    if (grabber->parentItem())
    {
      LineSegmentItem *wallSegment =
            dynamic_cast<LineSegmentItem *>(grabber->parentItem());
      if (wallSegment)
      {
        QLineF segmentLine(wallSegment->line());
        segmentLine.setP1(wallSegment->mapToScene(segmentLine.p1()));
        segmentLine.setP2(wallSegment->mapToScene(segmentLine.p2()));
        QPointF mousePoint = this->mapToScene(_event->pos());
        QPointF deltaLine = segmentLine.p2() - segmentLine.p1();
        QPointF deltaMouse = mousePoint - segmentLine.p1();
        double deltaLineMouse2 = deltaLine.x()*(-deltaMouse.y())
            - (-deltaMouse.x())*deltaLine.y();
        double deltaLine2 = (deltaLine.x()*deltaLine.x())
            + deltaLine.y()*deltaLine.y();
        double mouseDotLine = deltaMouse.x()*deltaLine.x()
              + deltaMouse.y()*deltaLine.y();
        double t = mouseDotLine / deltaLine2;
        double distance = fabs(deltaLineMouse2) / sqrt(deltaLine2);
        if (distance > 30 || t > 1.0 || t < 0.0)
        {
          editorItem->setParentItem(NULL);
          this->buildingMaker->DetachObject(this->itemToModelMap[editorItem],
                this->itemToModelMap[wallSegment]);
          editorItem->SetRotation(editorItem->GetRotation()
            - this->grabberDragRotation);
          editorItem->SetPosition(mousePoint);
        }
        else
        {
          QPointF closest(segmentLine.p1() + t*deltaLine);
          grabber->setPos(wallSegment->mapFromScene(closest));
          grabber->setRotation(wallSegment->rotation());
        }
        return;
      }
    }
    else
    {
      QList<QGraphicsItem *> overlaps = this->scene()->collidingItems(
          grabber, Qt::IntersectsItemBoundingRect);
      for (int i = 0; i < overlaps.size(); ++i)
      {
        LineSegmentItem *wallSegment =
            dynamic_cast<LineSegmentItem *>(overlaps[i]);
        if (wallSegment)
        {
          QPointF scenePos =  grabber->scenePos();
          if (wallSegment->contains(wallSegment->mapFromScene(scenePos)))
          {
            editorItem->setParentItem(wallSegment);
            this->buildingMaker->AttachObject(
                this->itemToModelMap[editorItem],
                this->itemToModelMap[wallSegment]);
            editorItem->SetPosition(wallSegment->mapFromScene(scenePos));
            this->grabberDragRotation = -wallSegment->line().angle();
            editorItem->SetRotation(this->grabberDragRotation);
            return;
          }
        }
      }
    }
  }

  if (!drawInProgress)
  {
    QGraphicsView::mouseMoveEvent(_event);
  }
}

/////////////////////////////////////////////////
void EditorView::keyPressEvent(QKeyEvent *_event)
{
  if (_event->key() == Qt::Key_Delete || _event->key() == Qt::Key_Backspace)
    {
    QList<QGraphicsItem *> selectedItems = this->scene()->selectedItems();

    for (int i = 0; i < selectedItems.size(); ++i)
    {
      EditorItem *item = dynamic_cast<EditorItem *>(selectedItems[i]);
      if (item)
      {
        if (item->GetType() == "Wall")
        {
          wallList.erase(std::remove(wallList.begin(), wallList.end(),
              dynamic_cast<WallItem *>(item)), wallList.end());
        }
        else if (item->GetType() == "Window")
        {
          windowList.erase(std::remove(windowList.begin(), windowList.end(),
              dynamic_cast<WindowItem *>(item)), windowList.end());
        }
        else if (item->GetType() == "Door")
        {
          doorList.erase(std::remove(doorList.begin(), doorList.end(),
              dynamic_cast<DoorItem *>(item)), doorList.end());
        }
        else if (item->GetType() == "Stairs")
        {
          stairsList.erase(std::remove(stairsList.begin(), stairsList.end(),
              dynamic_cast<StairsItem *>(item)), stairsList.end());
        }
        else if (item->GetType() == "Floor")
        {
          floorList.erase(std::remove(floorList.begin(), floorList.end(),
              dynamic_cast<FloorItem *>(item)), floorList.end());
        }

        if (item->GetType() == "Line")
        {
          EditorItem *itemParent = dynamic_cast<EditorItem *>(
              selectedItems[i]->parentItem());
          wallList.erase(std::remove(wallList.begin(), wallList.end(),
              dynamic_cast<WallItem *>(itemParent)), wallList.end());
          this->itemToModelMap.erase(item);
          delete itemParent;
        }
        else {
          this->itemToModelMap.erase(item);
          delete selectedItems[i];
        }
      }
    }
    this->drawMode = NONE;
    this->drawInProgress = false;
    this->currentMouseItem = NULL;
    QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
  }
  else if (_event->key() == Qt::Key_Escape)
  {
    this->CancelDrawMode();
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
    this->currentMouseItem = NULL;
    this->drawMode = NONE;
    this->drawInProgress = false;
    QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
  }
  else
  {
    if (!this->scene()->itemAt(this->mapToScene(_event->pos())))
      this->OnOpenLevelInspector();
  }

  if (!this->drawInProgress)
    QGraphicsView::mouseDoubleClickEvent(_event);
}

/////////////////////////////////////////////////
void EditorView::DrawWall(const QPoint &_pos)
{
  WallItem *wallItem = NULL;
  if (!drawInProgress)
  {
    QPointF pointStart = mapToScene(_pos);
    QPointF pointEnd = pointStart + QPointF(1, 0);

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

      wallItem->AddPoint(segment->mapToScene(segment->line().p2())
          + QPointF(1, 0));
    }
  }

  if (wallItem)
  {
    LineSegmentItem *segment = wallItem->GetSegment(
        wallItem->GetSegmentCount()-1);

    QVector3D segmentPosition = segment->GetScenePosition();
    segmentPosition.setZ(wallItem->GetLevelBaseHeight() + segmentPosition.z());
    QVector3D segmentSize = segment->GetSize();
    segmentSize.setZ(wallItem->GetHeight());
    std::string wallSegmentName = this->buildingMaker->AddWall(
        segmentSize, segmentPosition, segment->GetSceneRotation());
    this->lastWallSegmentName = wallSegmentName;
    this->buildingMaker->ConnectItem(wallSegmentName, segment);
    this->buildingMaker->ConnectItem(wallSegmentName, wallItem);
    this->itemToModelMap[segment] = wallSegmentName;
//    this->itemToModelMap[wallItem] = wallSegmentName;
  }
}

/////////////////////////////////////////////////
void EditorView::DrawWindow(const QPoint &_pos)
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
    this->itemToModelMap[windowItem] = windowName;

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
void EditorView::DrawDoor(const QPoint &_pos)
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
    this->itemToModelMap[doorItem] = doorName;

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
void EditorView::DrawStairs(const QPoint &_pos)
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
    this->itemToModelMap[stairsItem] = stairsName;

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
void EditorView::CreateItem3D(EditorItem* _item)
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
    this->itemToModelMap[stairsItem] = stairsName;
    if (stairsItem->GetLevel() < static_cast<int>(floorList.size()))
    {
      this->buildingMaker->AttachObject(stairsName,
          this->itemToModelMap[floorList[stairsItem->GetLevel()]]);
    }
  }
  //TODO add implementation for other times
}

/////////////////////////////////////////////////
void EditorView::OnCreateEditorItem(const std::string &_type)
{
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

  if (this->drawMode == WALL)
    QApplication::setOverrideCursor(QCursor(Qt::CrossCursor));
}

/////////////////////////////////////////////////
void EditorView::OnSaveModel(const std::string &_modelName,
    const std::string &_savePath)
{
  this->buildingMaker->SetModelName(_modelName);
  this->buildingMaker->GenerateSDF();
  this->buildingMaker->SaveToSDF(_savePath);
}

/////////////////////////////////////////////////
void EditorView::OnFinishModel()
{
  this->buildingMaker->FinishModel();
}

/////////////////////////////////////////////////
void EditorView::OnDiscardModel()
{
  this->wallList.clear();
  this->windowList.clear();
  this->doorList.clear();
  this->stairsList.clear();
  this->floorList.clear();
  this->itemToModelMap.clear();
  this->buildingMaker->Reset();
  this->levelNames.clear();
  this->levelHeights.clear();

  this->levelHeights[0] = 0;
  this->levelNames[0] = "Level 1";

  // clear the scene but add the grid back in
  this->scene()->clear();
  GridLines *gridLines = new GridLines (this->scene()->sceneRect().width(),
      this->scene()->sceneRect().height());
  this->scene()->addItem(gridLines);

  this->currentMouseItem = NULL;
  this->drawInProgress = false;
  this->drawMode = NONE;
}

/////////////////////////////////////////////////
void EditorView::OnAddLevel()
{
  int newLevel = this->levelHeights.size();
  std::stringstream levelNameStr;
  levelNameStr << "Level " << newLevel + 1;
  std::string levelName = levelNameStr.str();
  this->currentLevel = newLevel;
  this->levelNames[newLevel] = levelName;
  emit gui::editor::Events::changeLevelName(this->currentLevel, levelName);
  if (wallList.size() == 0)
  {
    this->levelHeights[newLevel] = 0;
    return;
  }

  std::vector<WallItem *>::iterator wallIt = this->wallList.begin();
  double wallHeight = (*wallIt)->GetHeight() + (*wallIt)->GetLevelBaseHeight();
  double maxHeight = wallHeight;
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
  this->levelHeights[newLevel] = maxHeight;

  QPolygonF wallBound;
  std::vector<WallItem *> newWalls;
  for (std::vector<WallItem *>::iterator it = wallList.begin();
      it  != this->wallList.end(); ++it)
  {
    if ((*it)->GetLevel() != wallLevel)
      continue;

    WallItem *wallItem = (*it)->Clone();
    wallItem->SetLevel(newLevel);
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
      this->itemToModelMap[segment] = wallSegmentName;
//      this->itemToModelMap[wallItem] = wallSegmentName;
    }
    if (wallBound.isEmpty())
      wallBound = wallItem->mapToScene(wallItem->boundingRect());
    else wallBound.united(wallItem->mapToScene(wallItem->boundingRect()));
  }
  this->wallList.insert(this->wallList.end(), newWalls.begin(),
      newWalls.end());

  // FIXME: hack to at least get some floors going
  QRectF allWallBound = wallBound.boundingRect();
  QVector3D floorSize(allWallBound.width(), allWallBound.height(), 10);
  QVector3D floorPosition(allWallBound.x() + allWallBound.width()/2,
      allWallBound.y()+allWallBound.height()/2, this->levelHeights[newLevel]);

  FloorItem *floorItem = new FloorItem();
  floorItem->SetLevel(this->currentLevel);
  floorItem->SetLevelBaseHeight(this->levelHeights[this->currentLevel]);
  floorItem->setPos(floorPosition.x(), floorPosition.y());
  floorItem->SetSize(QSize(floorSize.x(), floorSize.y()));
  std::string floorName = this->buildingMaker->AddFloor(
      floorSize, floorPosition, 0);
  for (std::vector<StairsItem *>::iterator it = this->stairsList.begin();
      it != this->stairsList.end(); ++it)
  {
    if ((*it)->GetLevel() == (newLevel - 1))
      buildingMaker->AttachObject(this->itemToModelMap[(*it)], floorName);
  }
  this->buildingMaker->ConnectItem(floorName, floorItem);
  this->itemToModelMap[floorItem] = floorName;
  this->scene()->addItem(floorItem);
  this->floorList.push_back(floorItem);
}

/////////////////////////////////////////////////
void EditorView::OnChangeLevel(int _level)
{
  this->currentLevel = _level;
  for (std::vector<WallItem *>::iterator it = this->wallList.begin();
      it != this->wallList.end(); ++it)
  {
    if ((*it)->GetLevel() != _level)
      (*it)->setVisible(false);
    else (*it)->setVisible(true);
  }
  for (std::vector<WindowItem *>::iterator it = this->windowList.begin();
      it != this->windowList.end(); ++it)
  {
    if ((*it)->GetLevel() != _level)
      (*it)->setVisible(false);
    else (*it)->setVisible(true);
  }
  for (std::vector<DoorItem *>::iterator it = this->doorList.begin();
      it != this->doorList.end(); ++it)
  {
    if ((*it)->GetLevel() != _level)
      (*it)->setVisible(false);
    else (*it)->setVisible(true);
  }
  for (std::vector<StairsItem *>::iterator it = this->stairsList.begin();
      it != this->stairsList.end(); ++it)
  {
    if ((*it)->GetLevel() != _level && (*it)->GetLevel() != (_level - 1))
      (*it)->setVisible(false);
    else (*it)->setVisible(true);
  }
  for (std::vector<FloorItem *>::iterator it = this->floorList.begin();
      it != this->floorList.end(); ++it)
  {
    if ((*it)->GetLevel() != _level)
      (*it)->setVisible(false);
    else (*it)->setVisible(true);
  }
}

/////////////////////////////////////////////////
void EditorView::OnOpenLevelInspector()
{
  this->levelInspector->SetLevelName(this->levelNames[this->currentLevel]);
  this->levelInspector->show();
}

/////////////////////////////////////////////////
void EditorView::OnLevelApply()
{
  LevelInspectorDialog *dialog =
     qobject_cast<LevelInspectorDialog *>(QObject::sender());

  std::string newLevelName = dialog->GetLevelName();
    this->levelNames[this->currentLevel] = newLevelName;
    emit gui::editor::Events::changeLevelName(this->currentLevel, newLevelName);
}

/////////////////////////////////////////////////
void EditorView::CancelDrawMode()
{
  if (this->drawMode != NONE && this->currentMouseItem)
  {
    EditorItem *item = dynamic_cast<EditorItem *>(this->currentMouseItem);
    this->itemToModelMap.erase(item);
    if (drawMode == WALL)
    {
      WallItem* wallItem = dynamic_cast<WallItem*>(this->currentMouseItem);
      wallItem->PopEndPoint();
      wallList.push_back(wallItem);
      //this->buildingMaker->RemoveWall(this->lastWallSegmentName);
      this->lastWallSegmentName = "";
    }
    else
    {
      delete this->currentMouseItem;
    }

    this->drawMode = NONE;
    this->drawInProgress = false;
    this->currentMouseItem = NULL;
    QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
  }
}
