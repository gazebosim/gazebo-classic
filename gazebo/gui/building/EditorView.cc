/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include <boost/bind.hpp>

#include "gazebo/math/Angle.hh"

#include "gazebo/common/Color.hh"

#include "gazebo/gui/Conversions.hh"
#include "gazebo/gui/building/ImportImageDialog.hh"
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
#include "gazebo/gui/building/EditorViewPrivate.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
EditorView::EditorView(QWidget *_parent)
  : QGraphicsView(_parent), dataPtr(new EditorViewPrivate)
{
  this->setObjectName("editorView");

  this->dataPtr->drawMode = NONE;
  this->dataPtr->drawInProgress = false;
  this->dataPtr->floorplanVisible = true;
  this->dataPtr->elementsVisible = true;
  this->dataPtr->currentMouseItem = NULL;

  this->dataPtr->connections.push_back(
      gui::editor::Events::ConnectCreateBuildingEditorItem(
      boost::bind(&EditorView::OnCreateEditorItem, this, _1)));

  this->dataPtr->connections.push_back(
      gui::editor::Events::ConnectColorSelected(
      boost::bind(&EditorView::OnColorSelected, this, _1)));

  this->dataPtr->connections.push_back(
      gui::editor::Events::ConnectTextureSelected(
      boost::bind(&EditorView::OnTextureSelected, this, _1)));

  this->dataPtr->connections.push_back(
      gui::editor::Events::ConnectNewBuildingModel(
      boost::bind(&EditorView::OnDiscardModel, this)));

  this->dataPtr->connections.push_back(
      gui::editor::Events::ConnectAddBuildingLevel(
      boost::bind(&EditorView::OnAddLevel, this)));

  this->dataPtr->connections.push_back(
      gui::editor::Events::ConnectDeleteBuildingLevel(
      boost::bind(&EditorView::OnDeleteLevel, this)));

  this->dataPtr->connections.push_back(
      gui::editor::Events::ConnectChangeBuildingLevel(
      boost::bind(&EditorView::OnChangeLevel, this, _1)));

  this->dataPtr->connections.push_back(
      gui::editor::Events::ConnectShowFloorplan(
      boost::bind(&EditorView::OnShowFloorplan, this)));

  this->dataPtr->connections.push_back(
      gui::editor::Events::ConnectShowElements(
      boost::bind(&EditorView::OnShowElements, this)));

  this->dataPtr->mousePressRotation = 0;

  this->dataPtr->buildingMaker = new BuildingMaker();
  this->dataPtr->currentLevel = 0;
  this->dataPtr->levelDefaultHeight = 250;

  Level *newLevel = new Level();
  newLevel->level = 0;
  newLevel->baseHeight = 0;
  newLevel->height = this->dataPtr->levelDefaultHeight;
  newLevel->name = "Level 1";
  this->dataPtr->levels.push_back(newLevel);

  this->dataPtr->levelInspector = new LevelInspectorDialog();
  this->dataPtr->levelInspector->setModal(false);
  connect(this->dataPtr->levelInspector, SIGNAL(Applied()), this, SLOT(OnLevelApply()));

  this->dataPtr->openLevelInspectorAct = new QAction(tr("&Open Level Inspector"), this);
  this->dataPtr->openLevelInspectorAct->setStatusTip(tr("Open Level Inspector"));
  connect(this->dataPtr->openLevelInspectorAct, SIGNAL(triggered()),
    this, SLOT(OnOpenLevelInspector()));

  this->dataPtr->addLevelAct = new QAction(tr("&Add Level"), this);
  this->dataPtr->addLevelAct->setStatusTip(tr("Add Level"));
  connect(this->dataPtr->addLevelAct, SIGNAL(triggered()),
    this, SLOT(OnAddLevel()));

  this->dataPtr->deleteLevelAct = new QAction(tr("&Delete Level"), this);
  this->dataPtr->deleteLevelAct->setStatusTip(tr("Delete Level"));
  connect(this->dataPtr->deleteLevelAct, SIGNAL(triggered()),
    this, SLOT(OnDeleteLevel()));

  this->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
//  this->setRenderHint(QPainter::SmoothPixmapTransform);
//  this->setRenderHint(QPainter::Antialiasing);
  this->dataPtr->gridLines = NULL;

  this->dataPtr->viewScale = 1.0;
  this->dataPtr->levelCounter = 0;

  this->dataPtr->mouseTooltip = new QGraphicsTextItem;
  this->dataPtr->mouseTooltip->setPlainText(
      "Oops! Color and texture can only be added in the 3D view.");
  this->dataPtr->mouseTooltip->setZValue(10);
}

/////////////////////////////////////////////////
EditorView::~EditorView()
{
  if (this->dataPtr->buildingMaker)
    delete this->dataPtr->buildingMaker;

  delete this->dataPtr->levelInspector;
}

/////////////////////////////////////////////////
void EditorView::scrollContentsBy(int _dx, int _dy)
{
  QGraphicsView::scrollContentsBy(_dx, _dy);

  if (this->dataPtr->gridLines && this->scene())
  {
    this->dataPtr->gridLines->setPos(this->mapToScene(QPoint(this->width()/2,
        this->height()/2)));
  }
}

/////////////////////////////////////////////////
void EditorView::resizeEvent(QResizeEvent *_event)
{
  if (this->scene())
  {
    if (!this->dataPtr->gridLines)
    {
      this->dataPtr->gridLines = new GridLines(_event->size().width(),
          _event->size().height());
      this->scene()->addItem(this->dataPtr->gridLines);
    }
    else
    {
      this->dataPtr->gridLines->SetSize(_event->size().width(),
          _event->size().height());
    }
    this->dataPtr->gridLines->setPos(this->mapToScene(
        QPoint(_event->size().width()/2, _event->size().height()/2)));
  }
}

/////////////////////////////////////////////////
void EditorView::contextMenuEvent(QContextMenuEvent *_event)
{
  if (this->dataPtr->drawMode == COLOR || this->dataPtr->drawMode == TEXTURE)
    return;

  if (this->dataPtr->drawInProgress)
  {
    this->CancelDrawMode();
    gui::editor::Events::createBuildingEditorItem(std::string());
    _event->accept();
    return;
  }

  QGraphicsItem *item = this->scene()->itemAt(this->mapToScene(_event->pos()));
  if (item && item != this->dataPtr->levels[this->dataPtr->currentLevel]->backgroundPixmap)
  {
    _event->ignore();
    QGraphicsView::contextMenuEvent(_event);
    return;
  }

  QMenu menu(this);
  menu.addAction(this->dataPtr->addLevelAct);
  menu.addAction(this->dataPtr->deleteLevelAct);
  menu.addAction(this->dataPtr->openLevelInspectorAct);
  menu.exec(_event->globalPos());
  _event->accept();
}

/////////////////////////////////////////////////
void EditorView::wheelEvent(QWheelEvent *_event)
{
  int wheelIncr = 120;
  int sign = (_event->delta() > 0) ? 1 : -1;
  int delta = std::max(std::abs(_event->delta()), wheelIncr) * sign;
  int numSteps = delta / wheelIncr;

  QMatrix mat = matrix();
  QPointF mousePosition = _event->pos();

  mat.translate((width()/2) - mousePosition.x(), (height() / 2) -
    mousePosition.y());

  double scaleFactor = 1.15;

  if (numSteps > 0)
  {
    mat.scale(numSteps*scaleFactor, numSteps*scaleFactor);
    this->dataPtr->viewScale *= numSteps*scaleFactor;
  }
  else
  {
    mat.scale(-1/(numSteps*scaleFactor), -1/(numSteps*scaleFactor));
    this->dataPtr->viewScale *= -1/(numSteps*scaleFactor);
  }
  mat.translate(mousePosition.x() - (this->width()/2),
      mousePosition.y() -(this->height()/2));
  this->setMatrix(mat);

  if (this->dataPtr->gridLines)
  {
    this->dataPtr->gridLines->setPos(this->mapToScene(
        QPoint(this->width()/2, this->height()/2)));
  }

  gui::editor::Events::changeBuildingEditorZoom(this->dataPtr->viewScale);
  _event->accept();
}

/////////////////////////////////////////////////
void EditorView::mousePressEvent(QMouseEvent *_event)
{
  if (!this->dataPtr->drawInProgress && this->dataPtr->drawMode != WALL && this->dataPtr->drawMode != COLOR
      && this->dataPtr->drawMode != TEXTURE && (_event->button() != Qt::RightButton))
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
  if (this->dataPtr->drawMode == WALL)
  {
    this->DrawWall(Conversions::Convert(_event->pos()));
  }
  else if (this->dataPtr->drawMode != NONE)
  {
    if (this->dataPtr->drawInProgress)
    {
      if (this->dataPtr->drawMode == WINDOW)
      {
        this->dataPtr->windowList.push_back(dynamic_cast<WindowItem *>(
            this->dataPtr->currentMouseItem));
      }
      else if (this->dataPtr->drawMode == DOOR)
      {
        this->dataPtr->doorList.push_back(dynamic_cast<DoorItem *>(
            this->dataPtr->currentMouseItem));
      }
      else if (this->dataPtr->drawMode == STAIRS)
      {
        StairsItem *stairsItem = dynamic_cast<StairsItem *>(
            this->dataPtr->currentMouseItem);
        stairsItem->Set3dTexture(QString(""));
        stairsItem->Set3dColor(Qt::white);
        this->dataPtr->stairsList.push_back(stairsItem);
        if ((this->dataPtr->currentLevel) < static_cast<int>(this->dataPtr->floorList.size()))
        {
          this->dataPtr->buildingMaker->AttachManip(this->dataPtr->itemToVisualMap[stairsItem],
              this->dataPtr->itemToVisualMap[this->dataPtr->floorList[this->dataPtr->currentLevel]]);
        }
      }
      dynamic_cast<EditorItem *>(this->dataPtr->currentMouseItem)->
          SetHighlighted(false);

      this->dataPtr->drawMode = NONE;
      this->dataPtr->drawInProgress = false;
      gui::editor::Events::createBuildingEditorItem(std::string());
    }
  }

  if (!this->dataPtr->drawInProgress)
    this->dataPtr->currentMouseItem = NULL;

  QGraphicsView::mouseReleaseEvent(_event);
}

/////////////////////////////////////////////////
void EditorView::mouseMoveEvent(QMouseEvent *_event)
{
  switch (this->dataPtr->drawMode)
  {
    case NONE:
      break;
    case WALL:
    {
      WallSegmentItem *wallSegmentItem = dynamic_cast<WallSegmentItem *>(
          this->dataPtr->currentMouseItem);
      if (this->dataPtr->drawInProgress && wallSegmentItem)
      {
        this->dataPtr->snapToGrabber = false;
        this->dataPtr->snapGrabberOther = NULL;
        this->dataPtr->snapGrabberCurrent = NULL;

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
                this->dataPtr->snapGrabberOther = anotherWall->grabbers[0];
                this->dataPtr->snapGrabberCurrent = wallSegmentItem->grabbers[1];
                this->dataPtr->snapToGrabber = true;
                break;
              }
              else if (QVector2D(p2 - anotherWall->GetEndPoint()).length() <=
                  distanceToClose)
              {
                pf = anotherWall->GetEndPoint();
                this->dataPtr->snapGrabberOther = anotherWall->grabbers[1];
                this->dataPtr->snapGrabberCurrent = wallSegmentItem->grabbers[1];
                this->dataPtr->snapToGrabber = true;
                break;
              }
            }
          }

          if (!this->dataPtr->snapToGrabber)
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
      this->DrawWindow(Conversions::Convert(_event->pos()));
      break;
    case DOOR:
      this->DrawDoor(Conversions::Convert(_event->pos()));
      break;
    case STAIRS:
      this->DrawStairs(Conversions::Convert(_event->pos()));
      break;
    case COLOR:
    case TEXTURE:
    {
      if (!this->dataPtr->mouseTooltip->scene())
        this->scene()->addItem(this->dataPtr->mouseTooltip);

      this->dataPtr->mouseTooltip->setPos(this->mapToScene(_event->pos()) +
          QPointF(15, 15));
      break;
    }
    default:
      break;
  }

  // auto attach windows and doors to walls
  QGraphicsItem *grabber = this->scene()->mouseGrabberItem();
  if (!grabber)
    grabber = this->dataPtr->currentMouseItem;
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
          wallSegmentItem->setZValue(wallSegmentItem->zValueIdle);
          editorItem->SetPositionOnWall(0);
          editorItem->SetAngleOnWall(0);
          this->dataPtr->buildingMaker->DetachFromParent(
              this->dataPtr->itemToVisualMap[editorItem]);
          editorItem->SetRotation(editorItem->GetRotation()
            - this->dataPtr->mousePressRotation);
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
            wallSegmentItem->setZValue(wallSegmentItem->zValueIdle+5);
            this->dataPtr->buildingMaker->AttachManip(
                this->dataPtr->itemToVisualMap[editorItem],
                this->dataPtr->itemToVisualMap[wallSegmentItem]);
            editorItem->SetPosition(wallSegmentItem->mapFromScene(scenePos));
            this->dataPtr->mousePressRotation = -wallSegmentItem->line().angle();
            editorItem->SetRotation(this->dataPtr->mousePressRotation);

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

  if (!this->dataPtr->drawInProgress)
  {
    QGraphicsView::mouseMoveEvent(_event);
  }
}

/////////////////////////////////////////////////
void EditorView::leaveEvent(QEvent */*_event*/)
{
  if (this->dataPtr->mouseTooltip &&
      this->scene()->items().contains(this->dataPtr->mouseTooltip))
  {
    this->scene()->removeItem(this->dataPtr->mouseTooltip);
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
    this->dataPtr->drawMode = NONE;
    this->dataPtr->drawInProgress = false;
    this->dataPtr->currentMouseItem = NULL;
    this->releaseKeyboard();
    QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
    gui::editor::Events::createBuildingEditorItem(std::string());
  }
  else if (_event->key() == Qt::Key_Escape)
  {
    if (this->dataPtr->mouseTooltip &&
        this->scene()->items().contains(this->dataPtr->mouseTooltip))
      this->scene()->removeItem(this->dataPtr->mouseTooltip);
    this->CancelDrawMode();
    gui::editor::Events::createBuildingEditorItem(std::string());
    this->releaseKeyboard();
  }
}

/////////////////////////////////////////////////
void EditorView::mouseDoubleClickEvent(QMouseEvent *_event)
{
  if (this->dataPtr->drawMode == WALL)
  {
    WallSegmentItem *wallSegmentItem = dynamic_cast<WallSegmentItem *>(
        this->dataPtr->currentMouseItem);
    this->dataPtr->itemToVisualMap.erase(wallSegmentItem);

    this->UnlinkGrabbers(wallSegmentItem->grabbers[0]);

    this->scene()->removeItem(this->dataPtr->currentMouseItem);
    delete this->dataPtr->currentMouseItem;

    this->dataPtr->snapToGrabber = false;
    this->dataPtr->snapGrabberOther = NULL;
    this->dataPtr->snapGrabberCurrent = NULL;
    this->dataPtr->currentMouseItem = NULL;
    this->dataPtr->drawMode = NONE;
    this->dataPtr->drawInProgress = false;
    this->releaseKeyboard();
    QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
    gui::editor::Events::createBuildingEditorItem(std::string());
  }
  else if (this->dataPtr->drawMode == COLOR || this->dataPtr->drawMode == TEXTURE)
  {
    return;
  }
  else
  {
    if (!this->scene()->itemAt(this->mapToScene(_event->pos())))
      this->OnOpenLevelInspector();
  }

  if (!this->dataPtr->drawInProgress)
    QGraphicsView::mouseDoubleClickEvent(_event);
}

/////////////////////////////////////////////////
void EditorView::DeleteItem(EditorItem *_item)
{
  if (!_item)
    return;

  // To make holes in the final model, windows and doors are attached to walls,
  // and stairs are attached to floors above them.
  // Detach 3D manip, but 2D items may remain as children.
  this->dataPtr->buildingMaker->DetachAllChildren(this->dataPtr->itemToVisualMap[_item]);

  _item->SetHighlighted(false);

  if (_item->GetType() == "WallSegment")
  {
    WallSegmentItem *wallSegmentItem = dynamic_cast<WallSegmentItem *>(_item);

    // Delete item's child doors and windows before deleting item
    for (int i = wallSegmentItem->childItems().size()-1; i >=0; --i)
    {
      // WallSegmentItems have other children besides RectItems
      RectItem *rectItem = dynamic_cast<RectItem *>(
          wallSegmentItem->childItems().at(i));

      if (rectItem)
      {
        this->DeleteItem(rectItem);
      }
    }

    this->UnlinkGrabbers(wallSegmentItem->grabbers[0]);
    this->UnlinkGrabbers(wallSegmentItem->grabbers[1]);

    this->dataPtr->wallSegmentList.erase(std::remove(this->dataPtr->wallSegmentList.begin(),
        this->dataPtr->wallSegmentList.end(), dynamic_cast<WallSegmentItem *>(_item)),
        this->dataPtr->wallSegmentList.end());
  }
  else if (_item->GetType() == "Window")
  {
    this->dataPtr->windowList.erase(std::remove(this->dataPtr->windowList.begin(),
        this->dataPtr->windowList.end(), dynamic_cast<WindowItem *>(_item)),
        this->dataPtr->windowList.end());
  }
  else if (_item->GetType() == "Door")
  {
    this->dataPtr->doorList.erase(std::remove(this->dataPtr->doorList.begin(),
        this->dataPtr->doorList.end(), dynamic_cast<DoorItem *>(_item)),
        this->dataPtr->doorList.end());
  }
  else if (_item->GetType() == "Stairs")
  {
    this->dataPtr->stairsList.erase(std::remove(this->dataPtr->stairsList.begin(),
        this->dataPtr->stairsList.end(), dynamic_cast<StairsItem *>(_item)),
        this->dataPtr->stairsList.end());
  }
  else if (_item->GetType() == "Floor")
  {
    this->dataPtr->floorList.erase(std::remove(this->dataPtr->floorList.begin(),
        this->dataPtr->floorList.end(), dynamic_cast<FloorItem *>(_item)),
        this->dataPtr->floorList.end());
  }

  this->dataPtr->itemToVisualMap.erase(_item);
  delete _item;
}

/////////////////////////////////////////////////
void EditorView::DrawWall(const ignition::math::Vector2i &_pos)
{
  WallSegmentItem *wallSegmentItem = NULL;

  // First point on the chain
  if (!this->dataPtr->drawInProgress)
  {
    QPointF pointStart = mapToScene(Conversions::Convert(_pos));
    QPointF pointEnd = pointStart + QPointF(1, 0);

    wallSegmentItem = new WallSegmentItem(pointStart, pointEnd,
        this->dataPtr->levelDefaultHeight);

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

    wallSegmentItem->SetLevel(this->dataPtr->currentLevel);
    wallSegmentItem->SetLevelBaseHeight(this->dataPtr->levels[this->dataPtr->currentLevel]->
        baseHeight);
    this->scene()->addItem(wallSegmentItem);
    this->dataPtr->currentMouseItem = wallSegmentItem;
    this->dataPtr->drawInProgress = true;
  }
  // Subsequent points get linked in the chain
  else
  {
    // Snap currently finishing segment to a grabber in scene
    if (this->dataPtr->snapToGrabber && this->dataPtr->snapGrabberOther &&
        this->dataPtr->snapGrabberCurrent)
    {
      this->LinkGrabbers(this->dataPtr->snapGrabberOther, this->dataPtr->snapGrabberCurrent);
    }

    // Reset and start a new segment
    this->dataPtr->snapToGrabber = false;
    this->dataPtr->snapGrabberOther = NULL;
    this->dataPtr->snapGrabberCurrent = NULL;

    wallSegmentItem = dynamic_cast<WallSegmentItem*>(this->dataPtr->currentMouseItem);
    wallSegmentItem->Set3dTexture(QString(""));
    wallSegmentItem->Set3dColor(Qt::white);
    wallSegmentItem->SetHighlighted(false);
    this->dataPtr->wallSegmentList.push_back(wallSegmentItem);
    if (wallSegmentItem->GetLevel() > 0)
    {
      this->dataPtr->floorList[wallSegmentItem->GetLevel()-1]->AttachWallSegment(
          wallSegmentItem);
    }

    // Start from previous segment's end
    QPointF newPointStart = wallSegmentItem->GetEndPoint();
    QPointF newPointEnd = newPointStart + QPointF(1, 0);
    GrabberHandle *grabberStart = wallSegmentItem->grabbers[1];

    wallSegmentItem = new WallSegmentItem(newPointStart,
        newPointEnd, this->dataPtr->levelDefaultHeight);
    wallSegmentItem->SetLevel(this->dataPtr->currentLevel);
    wallSegmentItem->SetLevelBaseHeight(this->dataPtr->levels[
       this->dataPtr->currentLevel]->baseHeight);
    this->scene()->addItem(wallSegmentItem);
    this->dataPtr->currentMouseItem = wallSegmentItem;

    this->LinkGrabbers(grabberStart, wallSegmentItem->grabbers[0]);
  }

  // 3D counterpart
  if (wallSegmentItem)
    this->Create3DVisual(wallSegmentItem);
}

/////////////////////////////////////////////////
void EditorView::DrawWindow(const ignition::math::Vector2i &_pos)
{
  WindowItem *windowItem = NULL;
  if (!this->dataPtr->drawInProgress)
  {
    windowItem = new WindowItem();
    windowItem->SetLevel(this->dataPtr->currentLevel);
    windowItem->SetLevelBaseHeight(
        this->dataPtr->levels[this->dataPtr->currentLevel]->baseHeight);
    this->scene()->addItem(windowItem);
    this->dataPtr->currentMouseItem = windowItem;
    this->Create3DVisual(windowItem);
    this->dataPtr->drawInProgress = true;
  }
  windowItem = dynamic_cast<WindowItem*>(this->dataPtr->currentMouseItem);
  if (windowItem)
  {
    QPointF scenePos = this->mapToScene(Conversions::Convert(_pos));
    windowItem->SetPosition(scenePos.x(), scenePos.y());
  }
}

/////////////////////////////////////////////////
void EditorView::DrawDoor(const ignition::math::Vector2i &_pos)
{
  DoorItem *doorItem = NULL;
  if (!this->dataPtr->drawInProgress)
  {
    doorItem = new DoorItem();
    doorItem->SetLevel(this->dataPtr->currentLevel);
    doorItem->SetLevelBaseHeight(this->dataPtr->levels[this->dataPtr->currentLevel]->baseHeight);
    this->scene()->addItem(doorItem);
    this->dataPtr->currentMouseItem = doorItem;
    this->Create3DVisual(doorItem);
    this->dataPtr->drawInProgress = true;
  }
  doorItem = dynamic_cast<DoorItem*>(this->dataPtr->currentMouseItem);
  if (doorItem)
  {
    QPointF scenePos = this->mapToScene(Conversions::Convert(_pos));
    doorItem->SetPosition(scenePos.x(), scenePos.y());
  }
}

/////////////////////////////////////////////////
void EditorView::DrawStairs(const ignition::math::Vector2i &_pos)
{
  StairsItem *stairsItem = NULL;
  if (!this->dataPtr->drawInProgress)
  {
    stairsItem = new StairsItem();
    stairsItem->SetLevel(this->dataPtr->currentLevel);
    stairsItem->SetLevelBaseHeight(
        this->dataPtr->levels[this->dataPtr->currentLevel]->baseHeight);
    this->scene()->addItem(stairsItem);
    this->dataPtr->currentMouseItem = stairsItem;

    this->Create3DVisual(stairsItem);
    this->dataPtr->drawInProgress = true;
  }
  stairsItem = dynamic_cast<StairsItem*>(this->dataPtr->currentMouseItem);
  if (stairsItem)
  {
    QPointF scenePos = this->mapToScene(Conversions::Convert(_pos));
    stairsItem->SetPosition(scenePos.x(), scenePos.y());
  }
}

/////////////////////////////////////////////////
void EditorView::Create3DVisual(EditorItem *_item)
{
  QVector3D itemPosition = _item->GetScenePosition();
  itemPosition.setZ(_item->GetLevelBaseHeight() + itemPosition.z());
  QVector3D itemSize = _item->GetSize();
  std::string itemName;

  if (_item->GetType() == "Stairs")
  {
    StairsItem *stairsItem = dynamic_cast<StairsItem *>(_item);
    itemName = this->dataPtr->buildingMaker->AddStairs(
        itemSize, itemPosition, _item->GetSceneRotation(),
        stairsItem->GetSteps());
    if (stairsItem->GetLevel() < static_cast<int>(this->dataPtr->floorList.size()))
    {
      this->dataPtr->buildingMaker->AttachManip(itemName,
          this->dataPtr->itemToVisualMap[this->dataPtr->floorList[stairsItem->GetLevel()]]);
    }
  }
  else if (_item->GetType() == "WallSegment")
  {
    WallSegmentItem *wallSegmentItem = dynamic_cast<WallSegmentItem *>(_item);
    itemSize.setZ(wallSegmentItem->GetHeight());
    itemName = this->dataPtr->buildingMaker->AddWall(
        itemSize, itemPosition, _item->GetSceneRotation());
  }
  else if (_item->GetType() == "Window")
  {
    itemName = this->dataPtr->buildingMaker->AddWindow(
        itemSize, itemPosition, _item->GetSceneRotation());
  }
  else if (_item->GetType() == "Door")
  {
    itemName = this->dataPtr->buildingMaker->AddDoor(
        itemSize, itemPosition, _item->GetSceneRotation());
  }
  else if (_item->GetType() == "Floor")
  {
    itemName = this->dataPtr->buildingMaker->AddFloor(
        itemSize, _item->GetScenePosition(), 0);
  }
  this->dataPtr->buildingMaker->ConnectItem(itemName, _item);
  this->dataPtr->itemToVisualMap[_item] = itemName;
  _item->SetName(itemName);
}

/////////////////////////////////////////////////
void EditorView::SetBackgroundImage(const std::string &_filename,
    double _resolution)
{
  QImage img(QString::fromStdString(_filename));
  int newHeight = (img.height() * 100) / _resolution;
  img = img.scaledToHeight(newHeight);

  if (!this->dataPtr->levels[this->dataPtr->currentLevel]->backgroundPixmap)
  {
    this->dataPtr->levels[this->dataPtr->currentLevel]->backgroundPixmap =
      new QGraphicsPixmapItem(QPixmap::fromImage(img));
    this->scene()->addItem(this->dataPtr->levels[this->dataPtr->currentLevel]->backgroundPixmap);
  }
  else
  {
    this->dataPtr->levels[this->dataPtr->currentLevel]->backgroundPixmap->setPixmap(
        QPixmap::fromImage(img));
  }

  this->dataPtr->levels[this->dataPtr->currentLevel]->backgroundPixmap->
      setX(img.width() * -0.5);
  this->dataPtr->levels[this->dataPtr->currentLevel]->backgroundPixmap->
      setY(img.height() * -0.5);
  this->setSceneRect(img.width() * -0.5, img.height() * -0.5,
                     img.width(), img.height());

  if (!this->dataPtr->floorplanVisible)
  {
    this->dataPtr->levels[this->dataPtr->currentLevel]->backgroundPixmap->setVisible(false);
    gui::editor::Events::triggerShowFloorplan();
  }
  gui::editor::Events::createBuildingEditorItem(std::string());
}

/////////////////////////////////////////////////
void EditorView::OnCreateEditorItem(const std::string &_type)
{
  this->CancelDrawMode();

  if (_type == "wall")
    this->dataPtr->drawMode = WALL;
  else if (_type == "window")
    this->dataPtr->drawMode = WINDOW;
  else if (_type == "door")
    this->dataPtr->drawMode = DOOR;
  else if (_type == "stairs")
    this->dataPtr->drawMode = STAIRS;
  else if (_type == "image")
  {
    this->dataPtr->drawMode = NONE;
    ImportImageDialog *importImage = new ImportImageDialog(this);
    importImage->show();
  }

  if (this->dataPtr->drawInProgress && this->dataPtr->currentMouseItem)
  {
    this->scene()->removeItem(this->dataPtr->currentMouseItem);
    this->dataPtr->currentMouseItem = NULL;
    this->dataPtr->drawInProgress = false;
  }

  this->scene()->clearSelection();

  if (this->dataPtr->drawMode == WALL)
    QApplication::setOverrideCursor(QCursor(Qt::CrossCursor));

  if (!this->dataPtr->elementsVisible)
    gui::editor::Events::triggerShowElements();

  // this->grabKeyboard();
}

/////////////////////////////////////////////////
void EditorView::OnColorSelected(const common::Color &_color)
{
  QColor color = Conversions::Convert(_color);
  if (!color.isValid())
    return;

  this->CancelDrawMode();
  this->scene()->clearSelection();
  this->dataPtr->drawMode = COLOR;
}

/////////////////////////////////////////////////
void EditorView::OnTextureSelected(const std::string &_texture)
{
  if (_texture == "")
    return;

  this->CancelDrawMode();
  this->scene()->clearSelection();
  this->dataPtr->drawMode = TEXTURE;
}

/////////////////////////////////////////////////
void EditorView::OnDiscardModel()
{
  this->dataPtr->wallSegmentList.clear();
  this->dataPtr->windowList.clear();
  this->dataPtr->doorList.clear();
  this->dataPtr->stairsList.clear();
  this->dataPtr->floorList.clear();
  this->dataPtr->itemToVisualMap.clear();
  this->dataPtr->buildingMaker->Reset();

  for (unsigned int i = 0; i < this->dataPtr->levels.size(); ++i)
    delete this->dataPtr->levels[i];
  this->dataPtr->levels.clear();
  this->dataPtr->currentLevel = 0;

  Level *newLevel = new Level();
  newLevel->level = 0;
  newLevel->baseHeight = 0;
  newLevel->height = this->dataPtr->levelDefaultHeight;
  newLevel->name = "Level 1";
  this->dataPtr->levels.push_back(newLevel);
  this->dataPtr->levelCounter = 0;

  // clear the scene but add the grid back in
  this->scene()->clear();
  this->dataPtr->gridLines = new GridLines(this->scene()->sceneRect().width(),
      this->scene()->sceneRect().height());
  this->scene()->addItem(this->dataPtr->gridLines);

  this->dataPtr->currentMouseItem = NULL;
  this->dataPtr->drawInProgress = false;
  this->dataPtr->drawMode = NONE;
  gui::editor::Events::createBuildingEditorItem(std::string());
}

/////////////////////////////////////////////////
void EditorView::OnAddLevel()
{
  if (this->dataPtr->wallSegmentList.empty())
  {
    QMessageBox msgBox;
    msgBox.setText("Create new levels after adding walls.");
    msgBox.exec();
    return;
  }

  if (!this->dataPtr->elementsVisible)
    gui::editor::Events::triggerShowElements();

  if (this->dataPtr->levels[this->dataPtr->currentLevel]->backgroundPixmap)
    this->dataPtr->levels[this->dataPtr->currentLevel]->backgroundPixmap->setVisible(false);

  int newLevelNum = this->dataPtr->levels.size();
  std::stringstream levelNameStr;
  levelNameStr << "Level " << ++this->dataPtr->levelCounter + 1;
  std::string levelName = levelNameStr.str();
  this->dataPtr->currentLevel = newLevelNum;
  Level *newLevel = new Level();
  newLevel->name = levelName;
  newLevel->level = newLevelNum;
  newLevel->height = this->dataPtr->levelDefaultHeight;
  this->dataPtr->levels.push_back(newLevel);
  gui::editor::Events::updateLevelWidget(this->dataPtr->currentLevel, levelName);

  std::vector<WallSegmentItem *>::iterator wallIt =
      this->dataPtr->wallSegmentList.begin();
  double wallHeight = (*wallIt)->GetHeight() + (*wallIt)->GetLevelBaseHeight();
  double maxHeight = wallHeight;
  int wallLevel = 0;

  ++wallIt;
  for (wallIt; wallIt != this->dataPtr->wallSegmentList.end(); ++wallIt)
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
  this->dataPtr->levels[this->dataPtr->currentLevel]->floorItem = floorItem;
  std::vector<WallSegmentItem *> newWalls;
  std::map<WallSegmentItem *, WallSegmentItem *> clonedWallMap;
  for (std::vector<WallSegmentItem *>::iterator it = this->dataPtr->wallSegmentList.begin();
      it  != this->dataPtr->wallSegmentList.end(); ++it)
  {
    if ((*it)->GetLevel() != wallLevel)
      continue;

    WallSegmentItem *wallSegmentItem = (*it)->Clone();
    clonedWallMap[(*it)] = wallSegmentItem;
    wallSegmentItem->SetLevel(newLevelNum);
    wallSegmentItem->SetLevelBaseHeight(this->dataPtr->levels[this->dataPtr->currentLevel]->
        baseHeight);
    this->scene()->addItem(wallSegmentItem);
    newWalls.push_back(wallSegmentItem);

    this->Create3DVisual(wallSegmentItem);

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
      for (auto oldGrabber : oldWall->grabbers[g]->LinkedGrabbers())
      {
        WallSegmentItem *parentItem = dynamic_cast<WallSegmentItem*>(
            oldGrabber->parentItem());
        int index = oldGrabber->Index();

        newWall->grabbers[g]->PushLinkedGrabber(
            clonedWallMap[parentItem]->grabbers[index]);
      }
    }
  }

  this->dataPtr->wallSegmentList.insert(this->dataPtr->wallSegmentList.end(), newWalls.begin(),
      newWalls.end());

  floorItem->SetLevel(this->dataPtr->currentLevel);
  floorItem->SetLevelBaseHeight(this->dataPtr->levels[this->dataPtr->currentLevel]->baseHeight);
  this->Create3DVisual(floorItem);
  for (std::vector<StairsItem *>::iterator it = this->dataPtr->stairsList.begin();
      it != this->dataPtr->stairsList.end(); ++it)
  {
    if ((*it)->GetLevel() == (newLevelNum - 1))
    {
      this->dataPtr->buildingMaker->AttachManip(this->dataPtr->itemToVisualMap[(*it)],
          floorItem->GetName());
    }
  }
  this->scene()->addItem(floorItem);
  this->dataPtr->floorList.push_back(floorItem);
  floorItem->Set3dTexture(QString(""));
  floorItem->Set3dColor(Qt::white);
  floorItem->SetHighlighted(false);
}

/////////////////////////////////////////////////
void EditorView::OnDeleteLevel()
{
  this->DeleteLevel(this->dataPtr->currentLevel);
}

/////////////////////////////////////////////////
void EditorView::DeleteLevel(int _level)
{
  if (this->dataPtr->levels.size() == 1
      || _level >= static_cast<int>(this->dataPtr->levels.size()))
    return;

  // Delete current level and move to level below or above
  int newLevelIndex = _level - 1;
  if (newLevelIndex < 0)
    newLevelIndex = _level + 1;

  if (this->dataPtr->levels[_level]->backgroundPixmap)
  {
    this->scene()->removeItem(this->dataPtr->levels[_level]->backgroundPixmap);
    delete this->dataPtr->levels[_level]->backgroundPixmap;
    this->dataPtr->levels[_level]->backgroundPixmap = NULL;
  }

  this->OnChangeLevel(newLevelIndex);

  double deletedHeight = this->dataPtr->levels[_level]->height;
  std::vector<EditorItem *> toBeDeleted;
  for (std::vector<WindowItem *>::iterator it = this->dataPtr->windowList.begin();
      it != this->dataPtr->windowList.end(); ++it)
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
  for (std::vector<DoorItem *>::iterator it = this->dataPtr->doorList.begin();
      it != this->dataPtr->doorList.end(); ++it)
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
  for (std::vector<StairsItem *>::iterator it = this->dataPtr->stairsList.begin();
      it != this->dataPtr->stairsList.end(); ++it)
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
  for (std::vector<FloorItem *>::iterator it = this->dataPtr->floorList.begin();
      it != this->dataPtr->floorList.end(); ++it)
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
      this->dataPtr->wallSegmentList.begin();
      it != this->dataPtr->wallSegmentList.end(); ++it)
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
  for (unsigned int i = 0; i < this->dataPtr->levels.size(); ++i)
  {
    if (this->dataPtr->levels[i]->level == _level)
    {
      delete this->dataPtr->levels[i];
      levelNum = i;
    }
    else if (this->dataPtr->levels[i]->level > _level)
    {
      this->dataPtr->levels[i]->level--;
      this->dataPtr->levels[i]->baseHeight -= deletedHeight;
    }
  }
  this->dataPtr->levels.erase(this->dataPtr->levels.begin() + levelNum);
  this->dataPtr->currentLevel = newLevelIndex;

  gui::editor::Events::updateLevelWidget(_level, "");
}

/////////////////////////////////////////////////
void EditorView::OnChangeLevel(const int _level)
{
  if (_level < 0)
    return;

  if (this->dataPtr->levels[this->dataPtr->currentLevel]->backgroundPixmap)
    this->dataPtr->levels[this->dataPtr->currentLevel]->backgroundPixmap->setVisible(false);

  if (_level < static_cast<int>(this->dataPtr->levels.size()) &&
      this->dataPtr->levels[_level]->backgroundPixmap &&
      this->dataPtr->floorplanVisible)
  {
    this->dataPtr->levels[_level]->backgroundPixmap->setVisible(true);
  }

  this->dataPtr->currentLevel = _level;
  this->ShowCurrentLevelItems();
}

/////////////////////////////////////////////////
void EditorView::OnOpenLevelInspector()
{
  this->dataPtr->levelInspector->SetLevelName(this->dataPtr->levels[this->dataPtr->currentLevel]->name);
  FloorItem *floorItem = this->dataPtr->levels[this->dataPtr->currentLevel]->floorItem;
  if (floorItem)
  {
    this->dataPtr->levelInspector->ShowFloorWidget(true);
    this->dataPtr->levelInspector->SetColor(Conversions::Convert(
        floorItem->Get3dColor()));
    this->dataPtr->levelInspector->SetTexture(floorItem->Get3dTexture().toStdString());
  }
  else
  {
    this->dataPtr->levelInspector->ShowFloorWidget(false);
  }
  this->dataPtr->levelInspector->move(QCursor::pos());
  this->dataPtr->levelInspector->show();
}

/////////////////////////////////////////////////
void EditorView::OnLevelApply()
{
  LevelInspectorDialog *dialog =
      qobject_cast<LevelInspectorDialog *>(QObject::sender());

  std::string newLevelName = dialog->LevelName();
  this->dataPtr->levels[this->dataPtr->currentLevel]->name = newLevelName;
  FloorItem *floorItem = this->dataPtr->levels[this->dataPtr->currentLevel]->floorItem;
  if (floorItem)
  {
    floorItem->Set3dTexture(QString::fromStdString(dialog->Texture()));
    floorItem->Set3dColor(Conversions::Convert(dialog->Color()));
    floorItem->Set3dTransparency(0.4);
    floorItem->FloorChanged();
  }
  gui::editor::Events::updateLevelWidget(this->dataPtr->currentLevel, newLevelName);
}

/////////////////////////////////////////////////
void EditorView::CancelDrawMode()
{
  if (this->dataPtr->drawMode != NONE)
  {
    if (this->dataPtr->currentMouseItem)
    {
      EditorItem *item = dynamic_cast<EditorItem *>(this->dataPtr->currentMouseItem);
      this->dataPtr->itemToVisualMap.erase(item);

      WallSegmentItem *wallSegmentItem = dynamic_cast<WallSegmentItem *>(item);

      if (wallSegmentItem)
      {
        this->UnlinkGrabbers(wallSegmentItem->grabbers[0]);
      }
      this->scene()->removeItem(this->dataPtr->currentMouseItem);
      delete this->dataPtr->currentMouseItem;
    }
    this->dataPtr->snapToGrabber = false;
    this->dataPtr->snapGrabberOther = NULL;
    this->dataPtr->snapGrabberCurrent = NULL;
    this->dataPtr->drawMode = NONE;
    this->dataPtr->drawInProgress = false;
    this->dataPtr->currentMouseItem = NULL;
    QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
  }
}

/////////////////////////////////////////////////
void EditorView::OnShowFloorplan()
{
  this->dataPtr->floorplanVisible = !this->dataPtr->floorplanVisible;

  if (this->dataPtr->levels[this->dataPtr->currentLevel]->backgroundPixmap)
    this->dataPtr->levels[this->dataPtr->currentLevel]->backgroundPixmap->setVisible(
        !this->dataPtr->levels[this->dataPtr->currentLevel]->backgroundPixmap->isVisible());
}

/////////////////////////////////////////////////
void EditorView::OnShowElements()
{
  this->dataPtr->elementsVisible = !this->dataPtr->elementsVisible;

  this->ShowCurrentLevelItems();
}

/////////////////////////////////////////////////
void EditorView::ShowCurrentLevelItems()
{
  for (std::vector<WallSegmentItem *>::iterator it =
      this->dataPtr->wallSegmentList.begin(); it != this->dataPtr->wallSegmentList.end(); ++it)
  {
    if ((*it)->GetLevel() != this->dataPtr->currentLevel)
      (*it)->setVisible(false);
    else
      (*it)->setVisible(this->dataPtr->elementsVisible);
  }
  for (std::vector<WindowItem *>::iterator it = this->dataPtr->windowList.begin();
      it != this->dataPtr->windowList.end(); ++it)
  {
    if ((*it)->GetLevel() != this->dataPtr->currentLevel)
      (*it)->setVisible(false);
    else
      (*it)->setVisible(this->dataPtr->elementsVisible);
  }
  for (std::vector<DoorItem *>::iterator it = this->dataPtr->doorList.begin();
      it != this->dataPtr->doorList.end(); ++it)
  {
    if ((*it)->GetLevel() != this->dataPtr->currentLevel)
      (*it)->setVisible(false);
    else
      (*it)->setVisible(this->dataPtr->elementsVisible);
  }
  for (std::vector<StairsItem *>::iterator it = this->dataPtr->stairsList.begin();
      it != this->dataPtr->stairsList.end(); ++it)
  {
    if ((*it)->GetLevel() != this->dataPtr->currentLevel && (*it)->GetLevel() !=
        (this->dataPtr->currentLevel - 1))
      (*it)->setVisible(false);
    else
      (*it)->setVisible(this->dataPtr->elementsVisible);
  }
  for (std::vector<FloorItem *>::iterator it = this->dataPtr->floorList.begin();
      it != this->dataPtr->floorList.end(); ++it)
  {
    if ((*it)->GetLevel() != this->dataPtr->currentLevel)
      (*it)->setVisible(false);
    else
      (*it)->setVisible(this->dataPtr->elementsVisible);
  }
}

/////////////////////////////////////////////////
void EditorView::LinkGrabbers(GrabberHandle *_grabber1,
    GrabberHandle *_grabber2)
{
  if (_grabber1 && _grabber2 && _grabber1 != _grabber2)
  {
    // if _grabber2 is not yet linked to _grabber1
    auto linked1 = _grabber1->LinkedGrabbers();
    auto linked2 = _grabber2->LinkedGrabbers();
    if (std::find(linked1.begin(), linked1.end(), _grabber2) == linked1.end())
    {
      // Add _grabber2 so it moves when _grabber1 is moved
      _grabber1->PushLinkedGrabber(_grabber2);
      // also link _grabber1 to all grabbers already linked to _grabber2
      for (auto it : linked2)
      {
        this->LinkGrabbers(_grabber1, it);
      }
    }
    // if _grabber1 is not yet linked to _grabber2
    if (std::find(linked2.begin(), linked2.end(), _grabber1) == linked2.end())
    {
      // Add _grabber1 so it moves when _grabber2 is moved
      _grabber2->PushLinkedGrabber(_grabber1);
      // also link _grabber2 to all grabbers already linked to _grabber1
      for (auto it : linked1)
      {
        this->LinkGrabbers(_grabber2, it);
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
    for (auto linked : _grabber1->LinkedGrabbers())
    {
      linked->EraseLinkedGrabber(_grabber1);
    }
  }

  // TODO: add option to unlink grabbers besides when deleting one of them,
  // perhaps using a hot-key or at the wall inspector
}
