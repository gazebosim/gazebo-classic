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

#include "gazebo/math/Angle.hh"
#include "gazebo/gui/building/ImportImageDialog.hh"
#include "gazebo/gui/building/BuildingItem.hh"
#include "gazebo/gui/building/GridLines.hh"
#include "gazebo/gui/building/EditorItem.hh"
#include "gazebo/gui/building/RectItem.hh"
#include "gazebo/gui/building/WindowItem.hh"
#include "gazebo/gui/building/DoorItem.hh"
#include "gazebo/gui/building/StairsItem.hh"
#include "gazebo/gui/building/FloorItem.hh"
#include "gazebo/gui/building/GrabberHandle.hh"
#include "gazebo/gui/building/WallSegmentItem.hh"
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
  this->floorplanVisible = true;
  this->elementsVisible = true;

  this->connections.push_back(
  gui::editor::Events::ConnectCreateBuildingEditorItem(
    boost::bind(&EditorView::OnCreateEditorItem, this, _1)));

  this->connections.push_back(
      gui::editor::Events::ConnectColorSelected(
      boost::bind(&EditorView::OnColorSelected, this, _1)));

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
      gui::editor::Events::ConnectDeleteBuildingLevel(
      boost::bind(&EditorView::OnDeleteLevel, this)));

  this->connections.push_back(
      gui::editor::Events::ConnectChangeBuildingLevel(
      boost::bind(&EditorView::OnChangeLevel, this, _1)));

  this->connections.push_back(
      gui::editor::Events::ConnectShowFloorplan(
      boost::bind(&EditorView::OnShowFloorplan, this)));

  this->connections.push_back(
      gui::editor::Events::ConnectShowElements(
      boost::bind(&EditorView::OnShowElements, this)));

  this->mousePressRotation = 0;

  this->buildingMaker = new BuildingMaker();
  this->currentLevel = 0;
  this->levelDefaultHeight = 250;

  Level *newLevel = new Level();
  newLevel->level = 0;
  newLevel->baseHeight = 0;
  newLevel->height = this->levelDefaultHeight;
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

  this->mouseTooltip = new QGraphicsTextItem;
  this->mouseTooltip->setPlainText(
      "Oops! Color can only be added in the 3D view.");
  this->mouseTooltip->setVisible(false);
  this->mouseTooltip->setZValue(10);
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
    gui::editor::Events::createBuildingEditorItem(std::string());
    _event->accept();
    return;
  }

  QGraphicsItem *item = this->scene()->itemAt(this->mapToScene(_event->pos()));
  if (item && item != this->levels[this->currentLevel]->backgroundPixmap)
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
  if (this->drawMode == WALL)
  {
    this->DrawWall(_event->pos());
  }
  else if (this->drawMode != NONE)
  {
    if (this->drawInProgress)
    {
      if (this->drawMode == WINDOW)
      {
        this->windowList.push_back(dynamic_cast<WindowItem *>(
            this->currentMouseItem));
      }
      else if (this->drawMode == DOOR)
      {
        this->doorList.push_back(dynamic_cast<DoorItem *>(
            this->currentMouseItem));
      }
      else if (this->drawMode == STAIRS)
      {
        StairsItem *stairsItem = dynamic_cast<StairsItem *>(
            this->currentMouseItem);
        stairsItem->Set3dTexture(QString(""));
        stairsItem->Set3dColor(Qt::white);
        this->stairsList.push_back(stairsItem);
        if ((this->currentLevel) < static_cast<int>(floorList.size()))
        {
          this->buildingMaker->AttachManip(this->itemToVisualMap[stairsItem],
              this->itemToVisualMap[floorList[this->currentLevel]]);
        }
      }
      dynamic_cast<EditorItem *>(this->currentMouseItem)->
          SetHighlighted(false);

      this->drawMode = NONE;
      this->drawInProgress = false;
      gui::editor::Events::createBuildingEditorItem(std::string());
    }
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
      WallSegmentItem *wallSegmentItem = dynamic_cast<WallSegmentItem *>(
          this->currentMouseItem);
      if (this->drawInProgress && wallSegmentItem)
      {
        this->snapToGrabber = false;
        this->snapGrabberOther = NULL;
        this->snapGrabberCurrent = NULL;

        QPointF p1 = wallSegmentItem->line().p1();
        QPointF p2 = this->mapToScene(_event->pos());
        QPointF pf;
        pf = p2;

        if (!(QApplication::keyboardModifiers() & Qt::ShiftModifier))
        {
          double distanceToClose = 30;

          // Snap to other walls' points
          QList<QGraphicsItem *> itemsList = this->scene()->items(QRectF(
              QPointF(p2.x() - distanceToClose/2, p2.y() - distanceToClose/2),
              QPointF(p2.x() + distanceToClose/2, p2.y() + distanceToClose/2)));
          for (QList<QGraphicsItem *>::iterator it = itemsList.begin();
              it  != itemsList.end(); ++it)
          {
            WallSegmentItem *anotherWall = dynamic_cast<WallSegmentItem *>
                (*it);
            if (anotherWall && anotherWall != wallSegmentItem)
            {
              if (QVector2D(p2 - anotherWall->GetStartPoint()).length() <=
                  distanceToClose)
              {
                pf = anotherWall->GetStartPoint();
                this->snapGrabberOther = anotherWall->grabbers[0];
                this->snapGrabberCurrent = wallSegmentItem->grabbers[1];
                this->snapToGrabber = true;
                break;
              }
              else if (QVector2D(p2 - anotherWall->GetEndPoint()).length() <=
                  distanceToClose)
              {
                pf = anotherWall->GetEndPoint();
                this->snapGrabberOther = anotherWall->grabbers[1];
                this->snapGrabberCurrent = wallSegmentItem->grabbers[1];
                this->snapToGrabber = true;
                break;
              }
            }
          }

          if (!this->snapToGrabber)
          {
            // Snap to angular increments
            QLineF newLine(p1, p2);
            double angle = GZ_DTOR(QLineF(p1, p2).angle());
            double range = GZ_DTOR(SegmentItem::SnapAngle);
            int angleIncrement = angle / range;

            if ((angle - range*angleIncrement) > range*0.5)
              angleIncrement++;
            angle = -range*angleIncrement;

            // Snap to length increments
            double newLength = newLine.length();
            double lengthIncrement = SegmentItem::SnapLength /
                wallSegmentItem->GetScale();
            newLength  = round(newLength/lengthIncrement)*lengthIncrement-
                wallSegmentItem->GetThickness();

            pf.setX(p1.x() + qCos(angle)*newLength);
            pf.setY(p1.y() + qSin(angle)*newLength);
          }
        }
        wallSegmentItem->SetEndPoint(pf);
        wallSegmentItem->Update();
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
    case COLOR:
    {
      if (!this->mouseTooltip->scene())
        this->scene()->addItem(this->mouseTooltip);

      this->mouseTooltip->setVisible(true);
      this->mouseTooltip->setPos(this->mapToScene(_event->pos()) +
          QPointF(15, 15));
      break;
    }
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
      WallSegmentItem *wallSegmentItem =
            dynamic_cast<WallSegmentItem *>(grabber->parentItem());
      if (wallSegmentItem)
      {
        QLineF segmentLine(wallSegmentItem->line());
        segmentLine.setP1(wallSegmentItem->mapToScene(segmentLine.p1()));
        segmentLine.setP2(wallSegmentItem->mapToScene(segmentLine.p2()));
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
          editorItem->SetPositionOnWall(0);
          editorItem->SetAngleOnWall(0);
          this->buildingMaker->DetachManip(this->itemToVisualMap[editorItem],
                this->itemToVisualMap[wallSegmentItem]);
          editorItem->SetRotation(editorItem->GetRotation()
            - this->mousePressRotation);
          editorItem->SetPosition(mousePoint);
        }
        else
        {
          QPointF closest(segmentLine.p1() + t*deltaLine);
          grabber->setPos(wallSegmentItem->mapFromScene(closest));
          grabber->setRotation(wallSegmentItem->rotation());

          QPointF absPositionOnWall = grabber->pos() -
              wallSegmentItem->line().p1();
          double positionLength = sqrt(absPositionOnWall.x()*
                                       absPositionOnWall.x() +
                                       absPositionOnWall.y()*
                                       absPositionOnWall.y());
          editorItem->SetPositionOnWall(positionLength /
              wallSegmentItem->line().length());
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
        WallSegmentItem *wallSegmentItem =
            dynamic_cast<WallSegmentItem *>(overlaps[i]);
        if (wallSegmentItem)
        {
          QPointF scenePos =  grabber->scenePos();
          if (wallSegmentItem->contains(wallSegmentItem->mapFromScene(
              scenePos)))
          {
            editorItem->setParentItem(wallSegmentItem);
            this->buildingMaker->AttachManip(
                this->itemToVisualMap[editorItem],
                this->itemToVisualMap[wallSegmentItem]);
            editorItem->SetPosition(wallSegmentItem->mapFromScene(scenePos));
            this->mousePressRotation = -wallSegmentItem->line().angle();
            editorItem->SetRotation(this->mousePressRotation);

            QPointF absPositionOnWall = editorItem->pos() -
                wallSegmentItem->line().p1();
            double positionLength = sqrt(absPositionOnWall.x()*
                                         absPositionOnWall.x() +
                                         absPositionOnWall.y()*
                                         absPositionOnWall.y());
            editorItem->SetPositionOnWall(positionLength /
                wallSegmentItem->line().length());
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
void EditorView::leaveEvent(QEvent */*_event*/)
{
  this->mouseTooltip->setVisible(false);
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
    this->mouseTooltip->setVisible(false);
    this->CancelDrawMode();
    gui::editor::Events::createBuildingEditorItem(std::string());
    this->releaseKeyboard();
  }
}

/////////////////////////////////////////////////
void EditorView::mouseDoubleClickEvent(QMouseEvent *_event)
{
  if (this->drawMode == WALL)
  {
    WallSegmentItem *wallSegmentItem = dynamic_cast<WallSegmentItem *>(
        this->currentMouseItem);
    this->itemToVisualMap.erase(wallSegmentItem);

    this->UnlinkGrabbers(wallSegmentItem->grabbers[0]);

    this->scene()->removeItem(this->currentMouseItem);
    delete this->currentMouseItem;

    this->snapToGrabber = false;
    this->snapGrabberOther = NULL;
    this->snapGrabberCurrent = NULL;
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

  this->buildingMaker->DetachAllChildren(this->itemToVisualMap[_item]);

  if (_item->GetType() == "WallSegment")
  {
    WallSegmentItem *wallSegmentItem = dynamic_cast<WallSegmentItem *>(_item);
    this->UnlinkGrabbers(wallSegmentItem->grabbers[0]);
    this->UnlinkGrabbers(wallSegmentItem->grabbers[1]);

    this->wallSegmentList.erase(std::remove(this->wallSegmentList.begin(),
        this->wallSegmentList.end(), dynamic_cast<WallSegmentItem *>(_item)),
        this->wallSegmentList.end());
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

  this->itemToVisualMap.erase(_item);
  delete _item;
}

/////////////////////////////////////////////////
void EditorView::DrawWall(const QPoint &_pos)
{
  WallSegmentItem *wallSegmentItem = NULL;

  // First point on the chain
  if (!this->drawInProgress)
  {
    QPointF pointStart = mapToScene(_pos);
    QPointF pointEnd = pointStart + QPointF(1, 0);

    wallSegmentItem = new WallSegmentItem(pointStart, pointEnd,
        this->levelDefaultHeight);

    if (!(QApplication::keyboardModifiers() & Qt::ShiftModifier))
    {
      double distanceToClose = 30;

      // Snap to other walls' points
      QList<QGraphicsItem *> itemsList = this->scene()->items(QRectF(
          QPointF(pointStart.x() - distanceToClose/2,
                  pointStart.y() - distanceToClose/2),
          QPointF(pointStart.x() + distanceToClose/2,
                  pointStart.y() + distanceToClose/2)));
      for (QList<QGraphicsItem *>::iterator it = itemsList.begin();
          it  != itemsList.end(); ++it)
      {
        WallSegmentItem *anotherWall = dynamic_cast<WallSegmentItem *>(*it);
        if (anotherWall)
        {
          if (QVector2D(pointStart - anotherWall->GetStartPoint()).length()
              <= distanceToClose)
          {
            wallSegmentItem->SetStartPoint(anotherWall->GetStartPoint());
            this->LinkGrabbers(anotherWall->grabbers[0],
                wallSegmentItem->grabbers[0]);
            break;
          }
          else if (QVector2D(pointStart - anotherWall->GetEndPoint()).length()
              <= distanceToClose)
          {
            wallSegmentItem->SetStartPoint(anotherWall->GetEndPoint());
            this->LinkGrabbers(anotherWall->grabbers[1],
                wallSegmentItem->grabbers[0]);
            break;
          }
        }
      }
    }

    wallSegmentItem->SetLevel(this->currentLevel);
    wallSegmentItem->SetLevelBaseHeight(this->levels[this->currentLevel]->
        baseHeight);
    this->scene()->addItem(wallSegmentItem);
    this->currentMouseItem = wallSegmentItem;
    this->drawInProgress = true;
  }
  // Subsequent points get linked in the chain
  else
  {
    // Snap currently finishing segment to a grabber in scene
    if (this->snapToGrabber && this->snapGrabberOther &&
        this->snapGrabberCurrent)
    {
      this->LinkGrabbers(this->snapGrabberOther, this->snapGrabberCurrent);
    }

    // Reset and start a new segment
    this->snapToGrabber = false;
    this->snapGrabberOther = NULL;
    this->snapGrabberCurrent = NULL;

    wallSegmentItem = dynamic_cast<WallSegmentItem*>(this->currentMouseItem);
    wallSegmentItem->Set3dTexture(QString(""));
    wallSegmentItem->Set3dColor(Qt::white);
    wallSegmentItem->SetHighlighted(false);
    wallSegmentList.push_back(wallSegmentItem);
    if (wallSegmentItem->GetLevel() > 0)
    {
      floorList[wallSegmentItem->GetLevel()-1]->AttachWallSegment(
          wallSegmentItem);
    }

    // Start from previous segment's end
    QPointF newPointStart = wallSegmentItem->GetEndPoint();
    QPointF newPointEnd = newPointStart + QPointF(1, 0);
    GrabberHandle *grabberStart = wallSegmentItem->grabbers[1];

    wallSegmentItem = new WallSegmentItem(newPointStart,
        newPointEnd, this->levelDefaultHeight);
    wallSegmentItem->SetLevel(this->currentLevel);
    wallSegmentItem->SetLevelBaseHeight(this->levels[
       this->currentLevel]->baseHeight);
    this->scene()->addItem(wallSegmentItem);
    this->currentMouseItem = wallSegmentItem;

    this->LinkGrabbers(grabberStart, wallSegmentItem->grabbers[0]);
  }

  // 3D counterpart
  if (wallSegmentItem)
  {
    QVector3D segmentPosition = wallSegmentItem->GetScenePosition();
    segmentPosition.setZ(wallSegmentItem->GetLevelBaseHeight() +
        segmentPosition.z());
    QVector3D segmentSize = wallSegmentItem->GetSize();
    segmentSize.setZ(wallSegmentItem->GetHeight());
    std::string wallSegmentName = this->buildingMaker->AddWall(
        segmentSize, segmentPosition, wallSegmentItem->GetSceneRotation());
    this->buildingMaker->ConnectItem(wallSegmentName, wallSegmentItem);
    this->itemToVisualMap[wallSegmentItem] = wallSegmentName;
    wallSegmentItem->SetName(wallSegmentName);
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
    windowItem->SetLevelBaseHeight(
        this->levels[this->currentLevel]->baseHeight);
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
    doorItem->SetLevelBaseHeight(this->levels[this->currentLevel]->baseHeight);
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
    stairsItem->SetLevelBaseHeight(
        this->levels[this->currentLevel]->baseHeight);
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
void EditorView::SetBackgroundImage(const std::string &_filename,
    double _resolution)
{
  QImage img(QString::fromStdString(_filename));
  int newHeight = (img.height() * 100) / _resolution;
  img = img.scaledToHeight(newHeight);

  if (!this->levels[this->currentLevel]->backgroundPixmap)
  {
    this->levels[this->currentLevel]->backgroundPixmap =
      new QGraphicsPixmapItem(QPixmap::fromImage(img));
    this->scene()->addItem(this->levels[this->currentLevel]->backgroundPixmap);
  }
  else
  {
    this->levels[this->currentLevel]->backgroundPixmap->setPixmap(
        QPixmap::fromImage(img));
  }

  this->levels[this->currentLevel]->backgroundPixmap->
      setX(img.width() * -0.5);
  this->levels[this->currentLevel]->backgroundPixmap->
      setY(img.height() * -0.5);
  this->setSceneRect(img.width() * -0.5, img.height() * -0.5,
                     img.width(), img.height());

  if (!this->floorplanVisible)
  {
    this->levels[this->currentLevel]->backgroundPixmap->setVisible(false);
    gui::editor::Events::triggerShowFloorplan();
  }
  gui::editor::Events::createBuildingEditorItem(std::string());
}

/////////////////////////////////////////////////
void EditorView::OnCreateEditorItem(const std::string &_type)
{
  this->CancelDrawMode();

  if (_type == "wall")
    this->drawMode = WALL;
  else if (_type == "window")
    this->drawMode = WINDOW;
  else if (_type == "door")
    this->drawMode = DOOR;
  else if (_type == "stairs")
    this->drawMode = STAIRS;
  else if (_type == "image")
  {
    this->drawMode = NONE;
    ImportImageDialog *importImage = new ImportImageDialog(this);
    importImage->show();
  }

  if (this->drawInProgress && this->currentMouseItem)
  {
    this->scene()->removeItem(this->currentMouseItem);
    this->currentMouseItem = NULL;
    this->drawInProgress = false;
  }

  this->scene()->clearSelection();

  if (this->drawMode == WALL)
    QApplication::setOverrideCursor(QCursor(Qt::CrossCursor));

  if (!this->elementsVisible)
    gui::editor::Events::triggerShowElements();

  // this->grabKeyboard();
}

/////////////////////////////////////////////////
void EditorView::OnColorSelected(QColor _color)
{
  if (_color.isValid())
    this->drawMode = COLOR;
}

/////////////////////////////////////////////////
void EditorView::OnDiscardModel()
{
  this->wallSegmentList.clear();
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

  Level *newLevel = new Level();
  newLevel->level = 0;
  newLevel->baseHeight = 0;
  newLevel->height = this->levelDefaultHeight;
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
  if (this->wallSegmentList.empty())
  {
    QMessageBox msgBox;
    msgBox.setText("Create new levels after adding walls.");
    msgBox.exec();
    return;
  }

  if (!this->elementsVisible)
    gui::editor::Events::triggerShowElements();

  if (this->levels[this->currentLevel]->backgroundPixmap)
    this->levels[this->currentLevel]->backgroundPixmap->setVisible(false);

  int newLevelNum = this->levels.size();
  std::stringstream levelNameStr;
  levelNameStr << "Level " << ++this->levelCounter + 1;
  std::string levelName = levelNameStr.str();
  this->currentLevel = newLevelNum;
  Level *newLevel = new Level();
  newLevel->name = levelName;
  newLevel->level = newLevelNum;
  newLevel->height = this->levelDefaultHeight;
  this->levels.push_back(newLevel);
  gui::editor::Events::updateLevelWidget(this->currentLevel, levelName);

  std::vector<WallSegmentItem *>::iterator wallIt =
      this->wallSegmentList.begin();
  double wallHeight = (*wallIt)->GetHeight() + (*wallIt)->GetLevelBaseHeight();
  double maxHeight = wallHeight;
  int wallLevel = 0;

  ++wallIt;
  for (wallIt; wallIt != this->wallSegmentList.end(); ++wallIt)
  {
    wallHeight = (*wallIt)->GetHeight() + (*wallIt)->GetLevelBaseHeight();
    if ( wallHeight > maxHeight)
    {
      maxHeight = wallHeight;
      wallLevel = (*wallIt)->GetLevel();
    }
  }
  newLevel->baseHeight = maxHeight;

  FloorItem *floorItem = new FloorItem();
  this->levels[this->currentLevel]->floorItem = floorItem;
  std::vector<WallSegmentItem *> newWalls;
  std::map<WallSegmentItem *, WallSegmentItem *> clonedWallMap;
  for (std::vector<WallSegmentItem *>::iterator it = wallSegmentList.begin();
      it  != this->wallSegmentList.end(); ++it)
  {
    if ((*it)->GetLevel() != wallLevel)
      continue;

    WallSegmentItem *wallSegmentItem = (*it)->Clone();
    clonedWallMap[(*it)] = wallSegmentItem;
    wallSegmentItem->SetLevel(newLevelNum);
    wallSegmentItem->SetLevelBaseHeight(this->levels[this->currentLevel]->
        baseHeight);
    this->scene()->addItem(wallSegmentItem);
    newWalls.push_back(wallSegmentItem);

    QVector3D segmentSize = wallSegmentItem->GetSize();
    segmentSize.setZ(wallSegmentItem->GetHeight());
    QVector3D segmentPosition = wallSegmentItem->GetScenePosition();
    segmentPosition.setZ(wallSegmentItem->GetLevelBaseHeight()
        + segmentPosition.z());
    std::string wallSegmentName = this->buildingMaker->AddWall(
        segmentSize, segmentPosition, wallSegmentItem->GetSceneRotation());
    this->buildingMaker->ConnectItem(wallSegmentName, wallSegmentItem);
    this->itemToVisualMap[wallSegmentItem] = wallSegmentName;

    floorItem->AttachWallSegment(wallSegmentItem);
    wallSegmentItem->Set3dTexture(QString(""));
    wallSegmentItem->Set3dColor(Qt::white);
    wallSegmentItem->SetHighlighted(false);
  }

  // Clone linked grabber relations
  typedef std::map<WallSegmentItem *, WallSegmentItem *>::iterator clonedIt;
  for (clonedIt iterator = clonedWallMap.begin(); iterator !=
      clonedWallMap.end(); ++iterator)
  {
    WallSegmentItem *oldWall = iterator->first;
    WallSegmentItem *newWall = iterator->second;

    // start / end
    for (int g = 0; g < 2; ++g)
    {
      for (unsigned int i = 0; i < oldWall->grabbers[g]->linkedGrabbers.size();
          ++i)
      {
        WallSegmentItem *parentItem = dynamic_cast<WallSegmentItem*>(
            oldWall->grabbers[g]->linkedGrabbers[i]->parentItem());
        int index = oldWall->grabbers[g]->linkedGrabbers[i]->GetIndex();

        newWall->grabbers[g]->linkedGrabbers.push_back(
            clonedWallMap[parentItem]->grabbers[index]);
      }
    }
  }

  this->wallSegmentList.insert(this->wallSegmentList.end(), newWalls.begin(),
      newWalls.end());

  floorItem->SetLevel(this->currentLevel);
  floorItem->SetLevelBaseHeight(this->levels[this->currentLevel]->baseHeight);
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
  floorItem->Set3dTexture(QString(""));
  floorItem->Set3dColor(Qt::white);
  floorItem->SetHighlighted(false);
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

  // Delete current level and move to level below or above
  int newLevelIndex = _level - 1;
  if (newLevelIndex < 0)
    newLevelIndex = _level + 1;

  if (this->levels[_level]->backgroundPixmap)
  {
    this->scene()->removeItem(this->levels[_level]->backgroundPixmap);
    delete this->levels[_level]->backgroundPixmap;
    this->levels[_level]->backgroundPixmap = NULL;
  }

  this->OnChangeLevel(newLevelIndex);

  double deletedHeight = this->levels[_level]->height;
  std::vector<EditorItem *> toBeDeleted;
  for (std::vector<WindowItem *>::iterator it = this->windowList.begin();
      it != this->windowList.end(); ++it)
  {
    if ((*it)->GetLevel() == _level)
      toBeDeleted.push_back(*it);
    else if ((*it)->GetLevel() > _level)
    {
      (*it)->SetLevel((*it)->GetLevel()-1);
      (*it)->SetLevelBaseHeight((*it)->GetLevelBaseHeight() - deletedHeight);
      (*it)->WindowChanged();
    }
  }
  for (std::vector<DoorItem *>::iterator it = this->doorList.begin();
      it != this->doorList.end(); ++it)
  {
    if ((*it)->GetLevel() == _level)
      toBeDeleted.push_back(*it);
    else if ((*it)->GetLevel() > _level)
    {
      (*it)->SetLevel((*it)->GetLevel()-1);
      (*it)->SetLevelBaseHeight((*it)->GetLevelBaseHeight()-deletedHeight);
      (*it)->DoorChanged();
    }
  }
  for (std::vector<StairsItem *>::iterator it = this->stairsList.begin();
      it != this->stairsList.end(); ++it)
  {
    if ((*it)->GetLevel() == _level)
      toBeDeleted.push_back(*it);
    else if ((*it)->GetLevel() > _level)
    {
      (*it)->SetLevel((*it)->GetLevel()-1);
      (*it)->SetLevelBaseHeight((*it)->GetLevelBaseHeight()-deletedHeight);
      (*it)->StairsChanged();
    }
  }
  for (std::vector<FloorItem *>::iterator it = this->floorList.begin();
      it != this->floorList.end(); ++it)
  {
    if ((*it)->GetLevel() == _level)
      toBeDeleted.push_back(*it);
    else if ((*it)->GetLevel() > _level)
    {
      (*it)->SetLevel((*it)->GetLevel()-1);
      (*it)->SetLevelBaseHeight((*it)->GetLevelBaseHeight()-deletedHeight);
      (*it)->FloorChanged();
    }
  }
  for (std::vector<WallSegmentItem *>::iterator it =
      this->wallSegmentList.begin();
      it != this->wallSegmentList.end(); ++it)
  {
    if ((*it)->GetLevel() == _level)
      toBeDeleted.push_back(*it);
    else if ((*it)->GetLevel() > _level)
    {
      (*it)->SetLevel((*it)->GetLevel()-1);
      (*it)->SetLevelBaseHeight((*it)->GetLevelBaseHeight()-deletedHeight);
      (*it)->WallSegmentChanged();
    }
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
    }
    else if (this->levels[i]->level > _level)
    {
      this->levels[i]->level--;
      this->levels[i]->baseHeight -= deletedHeight;
    }
  }
  this->levels.erase(this->levels.begin() + levelNum);
  this->currentLevel = newLevelIndex;

  gui::editor::Events::updateLevelWidget(_level, "");
}

/////////////////////////////////////////////////
void EditorView::OnChangeLevel(int _level)
{
  if (_level < 0)
    return;

  if (this->levels[this->currentLevel]->backgroundPixmap)
    this->levels[this->currentLevel]->backgroundPixmap->setVisible(false);

  if (_level < static_cast<int>(this->levels.size()) &&
      this->levels[_level]->backgroundPixmap &&
      this->floorplanVisible)
  {
    this->levels[_level]->backgroundPixmap->setVisible(true);
  }

  this->currentLevel = _level;
  this->ShowCurrentLevelItems();
}

/////////////////////////////////////////////////
void EditorView::OnOpenLevelInspector()
{
  this->levelInspector->SetLevelName(this->levels[this->currentLevel]->name);
  FloorItem *floorItem = this->levels[this->currentLevel]->floorItem;
  if (floorItem)
  {
    this->levelInspector->floorWidget->show();
    this->levelInspector->SetFloorColor(floorItem->Get3dColor());
    this->levelInspector->SetFloorTexture(floorItem->Get3dTexture());
  }
  else
  {
    this->levelInspector->floorWidget->hide();
  }
  this->levelInspector->move(QCursor::pos());
  this->levelInspector->show();
}

/////////////////////////////////////////////////
void EditorView::OnLevelApply()
{
  LevelInspectorDialog *dialog =
      qobject_cast<LevelInspectorDialog *>(QObject::sender());

  std::string newLevelName = dialog->GetLevelName();
  this->levels[this->currentLevel]->name = newLevelName;
  FloorItem *floorItem = this->levels[this->currentLevel]->floorItem;
  if (floorItem)
  {
    floorItem->Set3dTexture(dialog->GetFloorTexture());
    floorItem->Set3dColor(dialog->GetFloorColor());
    floorItem->Set3dTransparency(0.4);
    floorItem->FloorChanged();
  }
  gui::editor::Events::updateLevelWidget(this->currentLevel, newLevelName);
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

      WallSegmentItem *wallSegmentItem = dynamic_cast<WallSegmentItem *>(item);

      if (wallSegmentItem)
      {
        this->UnlinkGrabbers(wallSegmentItem->grabbers[0]);
      }
      this->scene()->removeItem(this->currentMouseItem);
      delete this->currentMouseItem;
    }
    this->snapToGrabber = false;
    this->snapGrabberOther = NULL;
    this->snapGrabberCurrent = NULL;
    this->drawMode = NONE;
    this->drawInProgress = false;
    this->currentMouseItem = NULL;
    QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
  }
}

/////////////////////////////////////////////////
void EditorView::OnShowFloorplan()
{
  this->floorplanVisible = !this->floorplanVisible;

  if (this->levels[this->currentLevel]->backgroundPixmap)
    this->levels[this->currentLevel]->backgroundPixmap->setVisible(
        !this->levels[this->currentLevel]->backgroundPixmap->isVisible());
}

/////////////////////////////////////////////////
void EditorView::OnShowElements()
{
  this->elementsVisible = !this->elementsVisible;

  this->ShowCurrentLevelItems();
}

/////////////////////////////////////////////////
void EditorView::ShowCurrentLevelItems()
{
  for (std::vector<WallSegmentItem *>::iterator it =
      this->wallSegmentList.begin(); it != this->wallSegmentList.end(); ++it)
  {
    if ((*it)->GetLevel() != this->currentLevel)
      (*it)->setVisible(false);
    else
      (*it)->setVisible(this->elementsVisible);
  }
  for (std::vector<WindowItem *>::iterator it = this->windowList.begin();
      it != this->windowList.end(); ++it)
  {
    if ((*it)->GetLevel() != this->currentLevel)
      (*it)->setVisible(false);
    else
      (*it)->setVisible(this->elementsVisible);
  }
  for (std::vector<DoorItem *>::iterator it = this->doorList.begin();
      it != this->doorList.end(); ++it)
  {
    if ((*it)->GetLevel() != this->currentLevel)
      (*it)->setVisible(false);
    else
      (*it)->setVisible(this->elementsVisible);
  }
  for (std::vector<StairsItem *>::iterator it = this->stairsList.begin();
      it != this->stairsList.end(); ++it)
  {
    if ((*it)->GetLevel() != this->currentLevel && (*it)->GetLevel() !=
        (this->currentLevel - 1))
      (*it)->setVisible(false);
    else
      (*it)->setVisible(this->elementsVisible);
  }
  for (std::vector<FloorItem *>::iterator it = this->floorList.begin();
      it != this->floorList.end(); ++it)
  {
    if ((*it)->GetLevel() != this->currentLevel)
      (*it)->setVisible(false);
    else
      (*it)->setVisible(this->elementsVisible);
  }
}

/////////////////////////////////////////////////
void EditorView::LinkGrabbers(GrabberHandle *_grabber1,
    GrabberHandle *_grabber2)
{
  if (_grabber1 && _grabber2 && _grabber1 != _grabber2)
  {
    // if _grabber2 is not yet linked to _grabber1
    if (std::find(_grabber1->linkedGrabbers.begin(),
                  _grabber1->linkedGrabbers.end(), _grabber2) ==
                  _grabber1->linkedGrabbers.end())
    {
      // Add _grabber2 so it moves when _grabber1 is moved
      _grabber1->linkedGrabbers.push_back(_grabber2);
      // also link _grabber1 to all grabbers already linked to _grabber2
      for (unsigned int i2 = 0; i2 < _grabber2->linkedGrabbers.size(); ++i2)
      {
        this->LinkGrabbers(_grabber1, _grabber2->linkedGrabbers[i2]);
      }
    }
    // if _grabber1 is not yet linked to _grabber2
    if (std::find(_grabber2->linkedGrabbers.begin(),
                  _grabber2->linkedGrabbers.end(), _grabber1) ==
                  _grabber2->linkedGrabbers.end())
    {
      // Add _grabber1 so it moves when _grabber2 is moved
      _grabber2->linkedGrabbers.push_back(_grabber1);
      // also link _grabber2 to all grabbers already linked to _grabber1
      for (unsigned int i1 = 0; i1 < _grabber1->linkedGrabbers.size(); ++i1)
      {
        this->LinkGrabbers(_grabber2, _grabber1->linkedGrabbers[i1]);
      }
    }
  }
}

/////////////////////////////////////////////////
void EditorView::UnlinkGrabbers(GrabberHandle *_grabber1,
    GrabberHandle *_grabber2)
{
  // If only one grabber, erase it from all grabbers it is linked
  if (!_grabber2)
  {
    for (unsigned int i = 0; i < _grabber1->linkedGrabbers.size(); ++i)
    {
      _grabber1->linkedGrabbers[i]->linkedGrabbers.erase(
          std::remove(_grabber1->linkedGrabbers[i]->linkedGrabbers.begin(),
          _grabber1->linkedGrabbers[i]->linkedGrabbers.end(), _grabber1),
          _grabber1->linkedGrabbers[i]->linkedGrabbers.end());
    }
  }

  // TODO: add option to unlink grabbers besides when deleting one of them,
  // perhaps using a hot-key or at the wall inspector
}
