/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include "gazebo/gui/building/BuildingItem.hh"
#include "gazebo/gui/building/GridLines.hh"
#include "gazebo/gui/building/EditorItem.hh"
#include "gazebo/gui/building/RectItem.hh"
#include "gazebo/gui/building/WindowItem.hh"
#include "gazebo/gui/building/DoorItem.hh"
#include "gazebo/gui/building/StairsItem.hh"
#include "gazebo/gui/building/FloorItem.hh"
#include "gazebo/gui/building/LineSegmentItem.hh"
#include "gazebo/gui/building/PolylineItem.hh"
#include "gazebo/gui/building/WallItem.hh"
#include "gazebo/gui/building/BuildingMaker.hh"
#include "gazebo/gui/building/LevelInspectorDialog.hh"
#include "gazebo/gui/building/BuildingEditorEvents.hh"
#include "gazebo/gui/building/EditorView.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
EditorView::EditorView(QWidget *_parent)
  : QGraphicsView(_parent), currentMouseItem(0)
{
  this->setObjectName("editorView");

  this->drawMode = NONE;
  this->drawInProgress = false;

  this->connections.push_back(
  gui::editor::Events::ConnectCreateBuildingEditorItem(
    boost::bind(&EditorView::OnCreateEditorItem, this, _1)));

/*  this->connections.push_back(
  gui::editor::Events::ConnectSaveModel(
    boost::bind(&EditorView::OnSaveModel, this, _1, _2)));*/

/*  this->connections.push_back(
  gui::editor::Events::ConnectDone(
    boost::bind(&EditorView::OnDone, this)));*/

  this->connections.push_back(
  gui::editor::Events::ConnectDiscardBuildingModel(
    boost::bind(&EditorView::OnDiscardModel, this)));

  this->connections.push_back(
  gui::editor::Events::ConnectAddBuildingLevel(
    boost::bind(&EditorView::OnAddLevel, this)));

  this->connections.push_back(
  gui::editor::Events::ConnectChangeBuildingLevel(
    boost::bind(&EditorView::OnChangeLevel, this, _1)));

  this->mousePressRotation = 0;

  this->buildingMaker = new BuildingMaker();
  this->currentLevel = 0;

  Level *newLevel = new Level;
  newLevel->level = 0;
  newLevel->height = 0;
  newLevel->name = "Level 1";
  this->levels.push_back(newLevel);

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

  this->deleteLevelAct = new QAction(tr("&Delete Level"), this);
  this->deleteLevelAct->setStatusTip(tr("Delete Level"));
  connect(this->deleteLevelAct, SIGNAL(triggered()),
    this, SLOT(OnDeleteLevel()));

  this->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
//  this->setRenderHint(QPainter::SmoothPixmapTransform);
//  this->setRenderHint(QPainter::Antialiasing);
  this->gridLines = NULL;

  this->viewScale = 1.0;
  this->levelCounter = 0;
}

/////////////////////////////////////////////////
EditorView::~EditorView()
{
  if (this->buildingMaker)
    delete this->buildingMaker;

  delete this->levelInspector;
}

/////////////////////////////////////////////////
void EditorView::scrollContentsBy(int _dx, int _dy)
{
  QGraphicsView::scrollContentsBy(_dx, _dy);

  if (this->gridLines && this->scene())
  {
    this->gridLines->setPos(this->mapToScene(QPoint(this->width()/2,
        this->height()/2)));
  }
}

/////////////////////////////////////////////////
void EditorView::resizeEvent(QResizeEvent *_event)
{
  if (this->scene())
  {
    if (!this->gridLines)
    {
      this->gridLines = new GridLines(_event->size().width(),
          _event->size().height());
      this->scene()->addItem(this->gridLines);
    }
    else
    {
      this->gridLines->SetSize(_event->size().width(),
          _event->size().height());
    }
    this->gridLines->setPos(this->mapToScene(
        QPoint(_event->size().width()/2, _event->size().height()/2)));
  }
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
  menu.addAction(this->deleteLevelAct);
  menu.addAction(this->openLevelInspectorAct);
  menu.exec(_event->globalPos());
  _event->accept();
}

/////////////////////////////////////////////////
void EditorView::wheelEvent(QWheelEvent *_event)
{
  int numSteps = (_event->delta()/8) / 15;

  QMatrix mat = matrix();
  QPointF mousePosition = _event->pos();

  mat.translate((width()/2) - mousePosition.x(), (height() / 2) -
    mousePosition.y());

  double scaleFactor = 1.15;

  if (numSteps > 0)
  {
    mat.scale(numSteps*scaleFactor, numSteps*scaleFactor);
    this->viewScale *= numSteps*scaleFactor;
  }
  else
  {
    mat.scale(-1/(numSteps*scaleFactor), -1/(numSteps*scaleFactor));
    this->viewScale *= -1/(numSteps*scaleFactor);
  }
  mat.translate(mousePosition.x() - (this->width()/2),
      mousePosition.y() -(this->height()/2));
  this->setMatrix(mat);

  if (this->gridLines)
  {
    this->gridLines->setPos(this->mapToScene(
        QPoint(this->width()/2, this->height()/2)));
  }

  gui::editor::Events::changeBuildingEditorZoom(this->viewScale);
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
  switch (this->drawMode)
  {
    case NONE:
      break;
    case WALL:
      this->DrawWall(_event->pos());
      break;
    case WINDOW:
      if (this->drawInProgress)
      {
        this->windowList.push_back(dynamic_cast<WindowItem *>(
            this->currentMouseItem));
        this->drawMode = NONE;
        this->drawInProgress = false;
        gui::editor::Events::createBuildingEditorItem(std::string());
      }
      break;
    case DOOR:
      if (this->drawInProgress)
      {
        this->doorList.push_back(dynamic_cast<DoorItem *>(
            this->currentMouseItem));
        this->drawMode = NONE;
        this->drawInProgress = false;
        gui::editor::Events::createBuildingEditorItem(std::string());
      }
      break;
    case STAIRS:
      if (this->drawInProgress)
      {
        this->stairsList.push_back(dynamic_cast<StairsItem *>(
            this->currentMouseItem));
        if ((this->currentLevel) < static_cast<int>(floorList.size()))
        {
          EditorItem *item = dynamic_cast<EditorItem *>(this->currentMouseItem);
          this->buildingMaker->AttachManip(this->itemToVisualMap[item],
              this->itemToVisualMap[floorList[this->currentLevel]]);
        }
        this->drawMode = NONE;
        this->drawInProgress = false;
        gui::editor::Events::createBuildingEditorItem(std::string());
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
  switch (this->drawMode)
  {
    case NONE:
      break;
    case WALL:
    {
      WallItem *wallItem = dynamic_cast<WallItem *>(this->currentMouseItem);
      if (this->drawInProgress && wallItem)
      {
        this->snapToCloseWall =false;
        if (wallItem->GetVertexCount() >= 3)
        {
          // snap end point to initial start point
          LineSegmentItem *segment = wallItem->GetSegment(0);
          QPointF firstPoint = segment->mapToScene(segment->line().p1());
          QPointF currentPoint = this->mapToScene(_event->pos());
          double distanceToClose = 30;
          if (QVector2D(currentPoint - firstPoint).length() <= distanceToClose)
          {
            wallItem->SetVertexPosition(wallItem->GetVertexCount()-1,
                firstPoint);
            this->snapToCloseWall = true;
          }
        }
        if (!snapToCloseWall)
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
      QApplication::setOverrideCursor(QCursor(Qt::CrossCursor));
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
          this->buildingMaker->DetachManip(this->itemToVisualMap[editorItem],
                this->itemToVisualMap[wallSegment]);
          editorItem->SetRotation(editorItem->GetRotation()
            - this->mousePressRotation);
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
            this->buildingMaker->AttachManip(
                this->itemToVisualMap[editorItem],
                this->itemToVisualMap[wallSegment]);
            editorItem->SetPosition(wallSegment->mapFromScene(scenePos));
            this->mousePressRotation = -wallSegment->line().angle();
            editorItem->SetRotation(this->mousePressRotation);
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
        this->DeleteItem(item);
      }
    }
    this->drawMode = NONE;
    this->drawInProgress = false;
    this->currentMouseItem = NULL;
    this->releaseKeyboard();
    QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
    gui::editor::Events::createBuildingEditorItem(std::string());
  }
  else if (_event->key() == Qt::Key_Escape)
  {
    this->CancelDrawMode();
    this->releaseKeyboard();
  }
}

/////////////////////////////////////////////////
void EditorView::mouseDoubleClickEvent(QMouseEvent *_event)
{
  if (this->drawMode == WALL)
  {
    WallItem* wallItem = dynamic_cast<WallItem*>(this->currentMouseItem);
    wallItem->PopEndPoint();
    if (this->snapToCloseWall)
    {
      wallItem->ClosePath();
      this->snapToCloseWall = false;
    }

    wallList.push_back(wallItem);
    // this->buildingMaker->RemoveWall(this->lastWallSegmentName);
    // this->lastWallSegmentName = "";
    if (wallItem->GetLevel() > 0)
      floorList[wallItem->GetLevel()-1]->AttachWall(wallItem);

    this->currentMouseItem = NULL;
    this->drawMode = NONE;
    this->drawInProgress = false;
    this->releaseKeyboard();
    QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
    gui::editor::Events::createBuildingEditorItem(std::string());
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
void EditorView::DeleteItem(EditorItem *_item)
{
  if (!_item)
    return;

  if (_item->GetType() == "Wall")
  {
    this->wallList.erase(std::remove(this->wallList.begin(),
        this->wallList.end(), dynamic_cast<WallItem *>(_item)),
        this->wallList.end());
  }
  else if (_item->GetType() == "Window")
  {
    this->windowList.erase(std::remove(this->windowList.begin(),
        this->windowList.end(), dynamic_cast<WindowItem *>(_item)),
        this->windowList.end());
  }
  else if (_item->GetType() == "Door")
  {
    this->doorList.erase(std::remove(this->doorList.begin(),
        this->doorList.end(), dynamic_cast<DoorItem *>(_item)),
        this->doorList.end());
  }
  else if (_item->GetType() == "Stairs")
  {
    this->stairsList.erase(std::remove(this->stairsList.begin(),
        this->stairsList.end(), dynamic_cast<StairsItem *>(_item)),
        this->stairsList.end());
  }
  else if (_item->GetType() == "Floor")
  {
    this->floorList.erase(std::remove(this->floorList.begin(),
        this->floorList.end(), dynamic_cast<FloorItem *>(_item)),
        this->floorList.end());
  }

  if (_item->GetType() == "Line")
  {
    QGraphicsItem *qItem = dynamic_cast<QGraphicsItem *>(_item);
    QGraphicsItem *itemParent = qItem->parentItem();
    this->wallList.erase(std::remove(this->wallList.begin(),
        this->wallList.end(), dynamic_cast<WallItem *>(itemParent)),
        this->wallList.end());
    this->itemToVisualMap.erase(_item);
    delete dynamic_cast<WallItem *>(itemParent);
  }
  else
  {
    this->itemToVisualMap.erase(_item);
    delete _item;
  }
}

/////////////////////////////////////////////////
void EditorView::DrawWall(const QPoint &_pos)
{
  WallItem *wallItem = NULL;
  if (!this->drawInProgress)
  {
    QPointF pointStart = mapToScene(_pos);
    QPointF pointEnd = pointStart + QPointF(1, 0);

    wallItem = new WallItem(pointStart, pointEnd);
    wallItem->SetLevel(this->currentLevel);
    wallItem->SetLevelBaseHeight(this->levels[this->currentLevel]->height);
    this->scene()->addItem(wallItem);
    this->currentMouseItem = wallItem;
    this->drawInProgress = true;
  }
  else
  {
    wallItem = dynamic_cast<WallItem *>(this->currentMouseItem);
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
    // this->lastWallSegmentName = wallSegmentName;
    this->buildingMaker->ConnectItem(wallSegmentName, segment);
    this->buildingMaker->ConnectItem(wallSegmentName, wallItem);
    this->itemToVisualMap[segment] = wallSegmentName;
    if (segment->GetIndex() == 0)
      wallItem->SetName(wallSegmentName);
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
    windowItem->SetLevelBaseHeight(this->levels[this->currentLevel]->height);
    this->scene()->addItem(windowItem);
    this->currentMouseItem = windowItem;

    QVector3D windowPosition = windowItem->GetScenePosition();
    windowPosition.setZ(windowItem->GetLevelBaseHeight() + windowPosition.z());
    std::string windowName = this->buildingMaker->AddWindow(
        windowItem->GetSize(), windowPosition, windowItem->GetSceneRotation());
    this->buildingMaker->ConnectItem(windowName, windowItem);
    this->itemToVisualMap[windowItem] = windowName;
    windowItem->SetName(windowName);
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
    doorItem->SetLevelBaseHeight(this->levels[this->currentLevel]->height);
    this->scene()->addItem(doorItem);
    this->currentMouseItem = doorItem;
    QVector3D doorPosition = doorItem->GetScenePosition();
    doorPosition.setZ(doorItem->GetLevelBaseHeight() + doorPosition.z());
    std::string doorName = this->buildingMaker->AddDoor(
        doorItem->GetSize(), doorPosition, doorItem->GetSceneRotation());
    this->buildingMaker->ConnectItem(doorName, doorItem);
    this->itemToVisualMap[doorItem] = doorName;
    doorItem->SetName(doorName);
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
    stairsItem->SetLevelBaseHeight(this->levels[this->currentLevel]->height);
    this->scene()->addItem(stairsItem);
    this->currentMouseItem = stairsItem;

    QVector3D stairsPosition = stairsItem->GetScenePosition();
    stairsPosition.setZ(stairsItem->GetLevelBaseHeight() + stairsPosition.z());
    std::string stairsName = this->buildingMaker->AddStairs(
        stairsItem->GetSize(), stairsPosition, stairsItem->GetSceneRotation(),
        stairsItem->GetSteps());

    this->buildingMaker->ConnectItem(stairsName, stairsItem);
    this->itemToVisualMap[stairsItem] = stairsName;
    stairsItem->SetName(stairsName);
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
void EditorView::Create3DVisual(EditorItem *_item)
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
    this->itemToVisualMap[stairsItem] = stairsName;
    stairsItem->SetName(stairsName);
    if (stairsItem->GetLevel() < static_cast<int>(floorList.size()))
    {
      this->buildingMaker->AttachManip(stairsName,
          this->itemToVisualMap[floorList[stairsItem->GetLevel()]]);
    }
  }
  // TODO add implementation for other times
}

/////////////////////////////////////////////////
void EditorView::OnCreateEditorItem(const std::string &_type)
{
  if (_type == "wall")
    this->drawMode = WALL;
  else if (_type == "window")
    this->drawMode = WINDOW;
  else if (_type == "door")
    this->drawMode = DOOR;
  else if (_type == "stairs")
    this->drawMode = STAIRS;
  else
    this->drawMode = NONE;

  if (this->drawInProgress && this->currentMouseItem)
  {
    this->scene()->removeItem(this->currentMouseItem);
    this->currentMouseItem = NULL;
    this->drawInProgress = false;
  }

  this->scene()->clearSelection();

  if (this->drawMode == WALL)
    QApplication::setOverrideCursor(QCursor(Qt::CrossCursor));

  // this->grabKeyboard();
}

/////////////////////////////////////////////////
void EditorView::OnDiscardModel()
{
  this->wallList.clear();
  this->windowList.clear();
  this->doorList.clear();
  this->stairsList.clear();
  this->floorList.clear();
  this->itemToVisualMap.clear();
  this->buildingMaker->Reset();

  for (unsigned int i = 0; i < this->levels.size(); ++i)
    delete this->levels[i];
  this->levels.clear();
  this->currentLevel = 0;

  Level *newLevel = new Level;
  newLevel->level = 0;
  newLevel->height = 0;
  newLevel->name = "Level 1";
  this->levels.push_back(newLevel);
  this->levelCounter = 0;

  // clear the scene but add the grid back in
  this->scene()->clear();
  this->gridLines = new GridLines(this->scene()->sceneRect().width(),
      this->scene()->sceneRect().height());
  this->scene()->addItem(this->gridLines);

  this->currentMouseItem = NULL;
  this->drawInProgress = false;
  this->drawMode = NONE;
  gui::editor::Events::createBuildingEditorItem(std::string());
}

/////////////////////////////////////////////////
void EditorView::OnAddLevel()
{
  int newLevelNum = this->levels.size();
  std::stringstream levelNameStr;
  levelNameStr << "Level " << ++this->levelCounter + 1;
  std::string levelName = levelNameStr.str();
  this->currentLevel = newLevelNum;
  Level *newlevel = new Level;
  newlevel->name = levelName;
  this->levels.push_back(newlevel);
  emit gui::editor::Events::changeBuildingLevelName(this->currentLevel,
      levelName);
  if (this->wallList.empty())
  {
    newlevel->height = 0;
    return;
  }

  std::vector<WallItem *>::iterator wallIt = this->wallList.begin();
  double wallHeight = (*wallIt)->GetHeight() + (*wallIt)->GetLevelBaseHeight();
  double maxHeight = wallHeight;
  int wallLevel = 0;

  ++wallIt;
  for (wallIt; wallIt != this->wallList.end(); ++wallIt)
  {
    wallHeight = (*wallIt)->GetHeight() + (*wallIt)->GetLevelBaseHeight();
    if ( wallHeight > maxHeight)
    {
      maxHeight = wallHeight;
      wallLevel = (*wallIt)->GetLevel();
    }
  }
  newlevel->height = maxHeight;

  FloorItem *floorItem = new FloorItem();
  std::vector<WallItem *> newWalls;
  for (std::vector<WallItem *>::iterator it = wallList.begin();
      it  != this->wallList.end(); ++it)
  {
    if ((*it)->GetLevel() != wallLevel)
      continue;

    WallItem *wallItem = (*it)->Clone();
    wallItem->SetLevel(newLevelNum);
    wallItem->SetLevelBaseHeight(this->levels[this->currentLevel]->height);
    this->scene()->addItem(wallItem);
    newWalls.push_back(wallItem);
    for (unsigned int j = 0; j < wallItem->GetSegmentCount(); ++j)
    {
      LineSegmentItem *segment = wallItem->GetSegment(j);
      QVector3D segmentSize = segment->GetSize();
      segmentSize.setZ(wallItem->GetHeight());
      QVector3D segmentPosition = segment->GetScenePosition();
      segmentPosition.setZ(wallItem->GetLevelBaseHeight()
          + segmentPosition.z());
      std::string wallSegmentName = this->buildingMaker->AddWall(
          segmentSize, segmentPosition, segment->GetSceneRotation());
      this->buildingMaker->ConnectItem(wallSegmentName, segment);
      this->buildingMaker->ConnectItem(wallSegmentName, wallItem);
      this->itemToVisualMap[segment] = wallSegmentName;
    }
    floorItem->AttachWall(wallItem);
  }
  this->wallList.insert(this->wallList.end(), newWalls.begin(),
      newWalls.end());

  floorItem->SetLevel(this->currentLevel);
  floorItem->SetLevelBaseHeight(this->levels[this->currentLevel]->height);
  std::string floorName = this->buildingMaker->AddFloor(
      floorItem->GetSize(), floorItem->GetScenePosition(), 0);
  for (std::vector<StairsItem *>::iterator it = this->stairsList.begin();
      it != this->stairsList.end(); ++it)
  {
    if ((*it)->GetLevel() == (newLevelNum - 1))
      buildingMaker->AttachManip(this->itemToVisualMap[(*it)], floorName);
  }
  this->buildingMaker->ConnectItem(floorName, floorItem);
  this->itemToVisualMap[floorItem] = floorName;
  this->scene()->addItem(floorItem);
  this->floorList.push_back(floorItem);
}

/////////////////////////////////////////////////
void EditorView::OnDeleteLevel()
{
  this->DeleteLevel(this->currentLevel);
}

/////////////////////////////////////////////////
void EditorView::DeleteLevel(int _level)
{
  if (this->levels.size() == 1
      || _level >= static_cast<int>(this->levels.size()))
    return;

  int newLevelIndex = _level - 1;
  if (newLevelIndex < 0)
    newLevelIndex = _level + 1;

  this->OnChangeLevel(newLevelIndex);
  std::vector<EditorItem *> toBeDeleted;
  for (std::vector<WindowItem *>::iterator it = this->windowList.begin();
      it != this->windowList.end(); ++it)
  {
    if ((*it)->GetLevel() == _level)
      toBeDeleted.push_back(*it);
    else if ((*it)->GetLevel() > _level)
      (*it)->SetLevel((*it)->GetLevel()-1);
  }
  for (std::vector<DoorItem *>::iterator it = this->doorList.begin();
      it != this->doorList.end(); ++it)
  {
    if ((*it)->GetLevel() == _level)
      toBeDeleted.push_back(*it);
    else if ((*it)->GetLevel() > _level)
      (*it)->SetLevel((*it)->GetLevel()-1);
  }
  for (std::vector<StairsItem *>::iterator it = this->stairsList.begin();
      it != this->stairsList.end(); ++it)
  {
    if ((*it)->GetLevel() == _level)
      toBeDeleted.push_back(*it);
    else if ((*it)->GetLevel() > _level)
      (*it)->SetLevel((*it)->GetLevel()-1);
  }
  for (std::vector<FloorItem *>::iterator it = this->floorList.begin();
      it != this->floorList.end(); ++it)
  {
    if ((*it)->GetLevel() == _level)
      toBeDeleted.push_back(*it);
    else if ((*it)->GetLevel() > _level)
      (*it)->SetLevel((*it)->GetLevel()-1);
  }
  for (std::vector<WallItem *>::iterator it = this->wallList.begin();
      it != this->wallList.end(); ++it)
  {
    if ((*it)->GetLevel() == _level)
      toBeDeleted.push_back(*it);
    else if ((*it)->GetLevel() > _level)
      (*it)->SetLevel((*it)->GetLevel()-1);
  }

  for (unsigned int i = 0; i < toBeDeleted.size(); ++i)
  {
    this->DeleteItem(toBeDeleted[i]);
  }

  int levelNum = 0;
  for (unsigned int i = 0; i < this->levels.size(); ++i)
  {
    if (this->levels[i]->level == _level)
    {
      delete this->levels[i];
      levelNum = i;
      break;
    }
    else if (this->levels[i]->level > _level)
    {
      this->levels[i]->level--;
    }
  }
  this->levels.erase(this->levels.begin() + levelNum);
  this->currentLevel = newLevelIndex;

  gui::editor::Events::deleteBuildingLevel(_level);
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
    else
      (*it)->setVisible(true);
  }
  for (std::vector<WindowItem *>::iterator it = this->windowList.begin();
      it != this->windowList.end(); ++it)
  {
    if ((*it)->GetLevel() != _level)
      (*it)->setVisible(false);
    else
      (*it)->setVisible(true);
  }
  for (std::vector<DoorItem *>::iterator it = this->doorList.begin();
      it != this->doorList.end(); ++it)
  {
    if ((*it)->GetLevel() != _level)
      (*it)->setVisible(false);
    else
      (*it)->setVisible(true);
  }
  for (std::vector<StairsItem *>::iterator it = this->stairsList.begin();
      it != this->stairsList.end(); ++it)
  {
    if ((*it)->GetLevel() != _level && (*it)->GetLevel() != (_level - 1))
      (*it)->setVisible(false);
    else
      (*it)->setVisible(true);
  }
  for (std::vector<FloorItem *>::iterator it = this->floorList.begin();
      it != this->floorList.end(); ++it)
  {
    if ((*it)->GetLevel() != _level)
      (*it)->setVisible(false);
    else
      (*it)->setVisible(true);
  }
}

/////////////////////////////////////////////////
void EditorView::OnOpenLevelInspector()
{
  this->levelInspector->SetLevelName(this->levels[this->currentLevel]->name);
  this->levelInspector->show();
}

/////////////////////////////////////////////////
void EditorView::OnLevelApply()
{
  LevelInspectorDialog *dialog =
     qobject_cast<LevelInspectorDialog *>(QObject::sender());

  std::string newLevelName = dialog->GetLevelName();
    this->levels[this->currentLevel]->name = newLevelName;
    emit gui::editor::Events::changeBuildingLevelName(this->currentLevel,
        newLevelName);
}

/////////////////////////////////////////////////
void EditorView::CancelDrawMode()
{
  if (this->drawMode != NONE)
  {
    if (this->currentMouseItem)
      {
      EditorItem *item = dynamic_cast<EditorItem *>(this->currentMouseItem);
      this->itemToVisualMap.erase(item);
      if (drawMode == WALL)
      {
        WallItem* wallItem = dynamic_cast<WallItem *>(this->currentMouseItem);
        wallItem->PopEndPoint();
        if (wallItem->GetVertexCount() >= 2)
        {
          wallList.push_back(wallItem);
          // this->buildingMaker->RemoveWall(this->lastWallSegmentName);
          // this->lastWallSegmentName = "";
          if (wallItem->GetLevel() > 0)
            floorList[wallItem->GetLevel()-1]->AttachWall(wallItem);
        }
        else
        {
          this->scene()->removeItem(this->currentMouseItem);
          delete wallItem;
        }
      }
      else
      {
        this->scene()->removeItem(this->currentMouseItem);
        delete this->currentMouseItem;
      }
    }
    this->snapToCloseWall = false;
    this->drawMode = NONE;
    this->drawInProgress = false;
    this->currentMouseItem = NULL;
    QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
    gui::editor::Events::createBuildingEditorItem(std::string());
  }
}
