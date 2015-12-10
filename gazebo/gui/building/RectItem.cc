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

#include "gazebo/math/Angle.hh"
#include "gazebo/gui/building/BuildingEditorWidget.hh"
#include "gazebo/gui/building/GrabberHandle.hh"
#include "gazebo/gui/building/RotateHandle.hh"
#include "gazebo/gui/building/EditorItem.hh"
#include "gazebo/gui/building/WallSegmentItem.hh"
#include "gazebo/gui/building/RectItem.hh"
#include "gazebo/gui/building/RectItemPrivate.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
RectItem::RectItem() : EditorItem(*new RectItemPrivate)
{
  auto dPtr = static_cast<RectItemPrivate *>(this->dataPtr);

  dPtr->editorType = "Rect";

  dPtr->width = 100;
  dPtr->height = 100;
  dPtr->highlighted = true;

  dPtr->drawingOriginX = 0;
  dPtr->drawingOriginY = 0;

  dPtr->positionOnWall = 0;
  dPtr->angleOnWall = 0;

  dPtr->drawingWidth = dPtr->width;
  dPtr->drawingHeight = dPtr->height;

  dPtr->borderColor = Qt::black;

  for (int i = 0; i < 8; ++i)
  {
    GrabberHandle *grabber = new GrabberHandle(this, i);
    dPtr->grabbers.push_back(grabber);
  }
  dPtr->rotateHandle = new RotateHandle(this);

  this->setSelected(false);
  this->setFlags(this->flags() | QGraphicsItem::ItemIsSelectable);
  this->setFlag(QGraphicsItem::ItemSendsScenePositionChanges);

  this->UpdateCornerPositions();
  this->setAcceptHoverEvents(true);

  dPtr->cursors.push_back(Qt::SizeFDiagCursor);
  dPtr->cursors.push_back(Qt::SizeVerCursor);
  dPtr->cursors.push_back(Qt::SizeBDiagCursor);
  dPtr->cursors.push_back(Qt::SizeHorCursor);

  this->setCursor(Qt::SizeAllCursor);

  dPtr->rotationAngle = 0;

  dPtr->zValueIdle = 1;
  dPtr->zValueSelected = 5;

  this->SetResizeFlag(ITEM_WIDTH | ITEM_HEIGHT);

  dPtr->openInspectorAct = new QAction(tr("&Open Inspector"), this);
  dPtr->openInspectorAct->setStatusTip(tr("Open Inspector"));
  connect(dPtr->openInspectorAct, SIGNAL(triggered()),
    this, SLOT(OnOpenInspector()));

  dPtr->deleteItemAct = new QAction(tr("&Delete"), this);
  dPtr->deleteItemAct->setStatusTip(tr("Delete"));
  connect(dPtr->deleteItemAct, SIGNAL(triggered()),
    this, SLOT(OnDeleteItem()));
}

//////////////////////////////////////////////////
RectItem::RectItem(RectItemPrivate &_dataPtr)
    : EditorItem(_dataPtr)
{
  auto dPtr = static_cast<RectItemPrivate *>(this->dataPtr);

  dPtr->editorType = "Rect";

  dPtr->width = 100;
  dPtr->height = 100;
  dPtr->highlighted = true;

  dPtr->drawingOriginX = 0;
  dPtr->drawingOriginY = 0;

  dPtr->positionOnWall = 0;
  dPtr->angleOnWall = 0;

  dPtr->drawingWidth = dPtr->width;
  dPtr->drawingHeight = dPtr->height;

  dPtr->borderColor = Qt::black;

  for (int i = 0; i < 8; ++i)
  {
    GrabberHandle *grabber = new GrabberHandle(this, i);
    dPtr->grabbers.push_back(grabber);
  }
  dPtr->rotateHandle = new RotateHandle(this);

  this->setSelected(false);
  this->setFlags(this->flags() | QGraphicsItem::ItemIsSelectable);
  this->setFlag(QGraphicsItem::ItemSendsScenePositionChanges);

  this->UpdateCornerPositions();
  this->setAcceptHoverEvents(true);

  dPtr->cursors.push_back(Qt::SizeFDiagCursor);
  dPtr->cursors.push_back(Qt::SizeVerCursor);
  dPtr->cursors.push_back(Qt::SizeBDiagCursor);
  dPtr->cursors.push_back(Qt::SizeHorCursor);

  this->setCursor(Qt::SizeAllCursor);

  dPtr->rotationAngle = 0;

  dPtr->zValueIdle = 1;
  dPtr->zValueSelected = 5;

  this->SetResizeFlag(ITEM_WIDTH | ITEM_HEIGHT);

  dPtr->openInspectorAct = new QAction(tr("&Open Inspector"), this);
  dPtr->openInspectorAct->setStatusTip(tr("Open Inspector"));
  connect(dPtr->openInspectorAct, SIGNAL(triggered()),
    this, SLOT(OnOpenInspector()));

  dPtr->deleteItemAct = new QAction(tr("&Delete"), this);
  dPtr->deleteItemAct->setStatusTip(tr("Delete"));
  connect(dPtr->deleteItemAct, SIGNAL(triggered()),
    this, SLOT(OnDeleteItem()));
}

/////////////////////////////////////////////////
RectItem::~RectItem()
{
  auto dPtr = static_cast<RectItemPrivate *>(this->dataPtr);

  for (int i = 0; i < 8; ++i)
  {
    dPtr->grabbers[i]->setParentItem(NULL);
    delete dPtr->grabbers[i];
  }
  dPtr->rotateHandle->setParentItem(NULL);
  delete dPtr->rotateHandle;
  if (!dPtr->measures.empty())
  {
    delete dPtr->measures[0];
    delete dPtr->measures[1];
  }
}

/////////////////////////////////////////////////
void RectItem::ShowHandles(bool _show)
{
  auto dPtr = static_cast<RectItemPrivate *>(this->dataPtr);

  for (int i = 0; i < 8; ++i)
  {
    dPtr->grabbers[i]->setVisible(_show && dPtr->grabbers[i]->isEnabled());
  }
  dPtr->rotateHandle->setVisible(_show);
}

/////////////////////////////////////////////////
void RectItem::AdjustSize(double _x, double _y)
{
  auto dPtr = static_cast<RectItemPrivate *>(this->dataPtr);

  dPtr->width += _x;
  dPtr->height += _y;
  dPtr->drawingWidth = dPtr->width;
  dPtr->drawingHeight = dPtr->height;
}

/////////////////////////////////////////////////
QVariant RectItem::itemChange(GraphicsItemChange _change,
  const QVariant &_value)
{
  if (_change == QGraphicsItem::ItemSelectedChange && this->scene())
  {
    this->SetHighlighted(_value.toBool());
  }
  else if (_change == QGraphicsItem::ItemScenePositionHasChanged
      && this->scene())
  {
    emit PosXChanged(this->scenePos().x());
    emit PosYChanged(this->scenePos().y());
  }
  return QGraphicsItem::itemChange(_change, _value);
}

/////////////////////////////////////////////////
void RectItem::SetHighlighted(bool _highlighted)
{
  auto dPtr = static_cast<RectItemPrivate *>(this->dataPtr);

  if (_highlighted)
  {
    this->setZValue(this->ZValueSelected());
    WallSegmentItem *wallItem = dynamic_cast<WallSegmentItem *>(
        this->parentItem());
    if (wallItem)
      wallItem->setZValue(wallItem->ZValueSelected());

    for (int i = 0; i < 8; ++i)
    {
      if (dPtr->grabbers[i]->isEnabled())
        dPtr->grabbers[i]->installSceneEventFilter(this);
    }
    dPtr->rotateHandle->installSceneEventFilter(this);
    this->Set3dTransparency(0.0);
  }
  else
  {
    this->setZValue(this->ZValueIdle());
    WallSegmentItem *wallItem = dynamic_cast<WallSegmentItem *>(
        this->parentItem());
    if (wallItem)
      wallItem->setZValue(wallItem->ZValueIdle());

    for (int i = 0; i < 8; ++i)
    {
      if (dPtr->grabbers[i]->isEnabled())
        dPtr->grabbers[i]->removeSceneEventFilter(this);
    }
    dPtr->rotateHandle->removeSceneEventFilter(this);
    this->Set3dTransparency(0.4);
  }
  dPtr->highlighted = _highlighted;
  this->UpdateMeasures();
}

/////////////////////////////////////////////////
bool RectItem::sceneEventFilter(QGraphicsItem * _watched, QEvent *_event)
{
  RotateHandle *rotateH = dynamic_cast<RotateHandle *>(_watched);
  if (rotateH != NULL)
    return this->RotateEventFilter(rotateH, _event);

  GrabberHandle *grabber = dynamic_cast<GrabberHandle *>(_watched);
  if (grabber != NULL && grabber->isEnabled())
    return this->GrabberEventFilter(grabber, _event);

  return false;
}

/////////////////////////////////////////////////
bool RectItem::RotateEventFilter(RotateHandle *_rotate, QEvent *_event)
{
  auto dPtr = static_cast<RectItemPrivate *>(this->dataPtr);

  QGraphicsSceneMouseEvent *mouseEvent =
    dynamic_cast<QGraphicsSceneMouseEvent*>(_event);

  switch (_event->type())
  {
    case QEvent::GraphicsSceneMousePress:
    {
      _rotate->SetMouseState(
          static_cast<int>(QEvent::GraphicsSceneMousePress));
      _rotate->SetMouseDownX(mouseEvent->pos().x());
      _rotate->SetMouseDownY(mouseEvent->pos().y());
      break;
    }
    case QEvent::GraphicsSceneMouseRelease:
    {
      _rotate->SetMouseState(
          static_cast<int>(QEvent::GraphicsSceneMouseRelease));
      break;
    }
    case QEvent::GraphicsSceneMouseMove:
    {
      _rotate->SetMouseState(static_cast<int>(QEvent::GraphicsSceneMouseMove));
      break;
    }
    case QEvent::GraphicsSceneHoverEnter:
    case QEvent::GraphicsSceneHoverMove:
    {
//      QApplication::setOverrideCursor(QCursor(Qt::CrossCursor));

      QApplication::setOverrideCursor(BuildingEditorWidget::rotateCursor);
      return true;
    }
    case QEvent::GraphicsSceneHoverLeave:
    {
      QApplication::restoreOverrideCursor();
      return true;
    }
    default:
      return false;
      break;
  }

  if (!mouseEvent)
    return false;

  if (_rotate->GetMouseState()
      == static_cast<int>(QEvent::GraphicsSceneMouseMove))
  {
    QPoint localCenter(dPtr->drawingOriginX, dPtr->drawingOriginY);
    QPointF center = this->mapToScene(localCenter);

    QPointF newPoint = mouseEvent->scenePos();
    QLineF line(center.x(), center.y(), newPoint.x(), newPoint.y());

    double angle = 0;

    if (this->parentItem())
    {
      QPointF localCenterTop(dPtr->drawingOriginX, dPtr->drawingOriginY
          + dPtr->drawingHeight);
      QPointF centerTop = this->mapToScene(localCenterTop);
      QLineF lineCenter(center.x(), center.y(), centerTop.x(), centerTop.y());
      angle = -lineCenter.angleTo(line);

      if (angle < 0)
        angle += 360;
      if (angle < 90 || angle > 270)
      {
        angle = 180;
        this->SetRotation(this->GetRotation() + angle);
        if (this->GetAngleOnWall() < 90)
          this->SetAngleOnWall(180);
        else
          this->SetAngleOnWall(0);
      }
    }
    else
    {
      QLineF prevLine(center.x(), center.y(),
          mouseEvent->lastScenePos().x(), mouseEvent->lastScenePos().y());
      angle = -prevLine.angleTo(line);
      this->SetRotation(this->GetRotation() + angle);
    }
  }
  return true;
}

/////////////////////////////////////////////////
bool RectItem::GrabberEventFilter(GrabberHandle *_grabber, QEvent *_event)
{
  auto dPtr = static_cast<RectItemPrivate *>(this->dataPtr);

  QGraphicsSceneMouseEvent *mouseEvent =
    dynamic_cast<QGraphicsSceneMouseEvent*>(_event);

  switch (_event->type())
  {
    case QEvent::GraphicsSceneMousePress:
    {
      _grabber->SetMouseState(
          static_cast<int>(QEvent::GraphicsSceneMousePress));
      _grabber->SetMouseDownX(mouseEvent->pos().x());
      _grabber->SetMouseDownY(mouseEvent->pos().y());
      break;
    }
    case QEvent::GraphicsSceneMouseRelease:
    {
      _grabber->SetMouseState(
          static_cast<int>(QEvent::GraphicsSceneMouseRelease));
      break;
    }
    case QEvent::GraphicsSceneMouseMove:
    {
      _grabber->SetMouseState(static_cast<int>(QEvent::GraphicsSceneMouseMove));
      break;
    }
    case QEvent::GraphicsSceneHoverEnter:
    case QEvent::GraphicsSceneHoverMove:
    {
      double angle = dPtr->rotationAngle
          - static_cast<int>(dPtr->rotationAngle/360) * 360;
      double range = 22.5;
      if (angle < 0)
        angle += 360;

      if ((angle > (360 - range)) || (angle < range)
          || ((angle <= (180 + range)) && (angle > (180 - range))))
      {
        QApplication::setOverrideCursor(
            QCursor(dPtr->cursors[_grabber->GetIndex() % 4]));
      }
      else if (((angle <= (360 - range)) && (angle > (270 + range)))
          || ((angle <= (180 - range)) && (angle > (90 + range))))
      {
        QApplication::setOverrideCursor(
            QCursor(dPtr->cursors[(_grabber->GetIndex() + 3) % 4]));
      }
      else if (((angle <= (270 + range)) && (angle > (270 - range)))
          || ((angle <= (90 + range)) && (angle > (90 - range))))
      {
        QApplication::setOverrideCursor(
            QCursor(dPtr->cursors[(_grabber->GetIndex() + 2) % 4]));
      }
      else
      {
        QApplication::setOverrideCursor(
            QCursor(dPtr->cursors[(_grabber->GetIndex() + 1) % 4]));
      }
      return true;
    }
    case QEvent::GraphicsSceneHoverLeave:
    {
      QApplication::restoreOverrideCursor();
      return true;
    }
    default:
      return false;
  }

  if (!mouseEvent)
    return false;


  if (_grabber->GetMouseState()
      == static_cast<int>(QEvent::GraphicsSceneMouseMove))
  {
    double xPos = mouseEvent->pos().x();
    double yPos = mouseEvent->pos().y();

    // depending on which grabber has been grabbed, we want to move the position
    // of the item as it grows/shrinks accordingly. so we need to either add
    // or subtract the offsets based on which grabber this is.

    int xAxisSign = 0;
    int yAxisSign = 0;
    switch (_grabber->GetIndex())
    {
      // corners
      case 0:
      {
        xAxisSign = 1;
        yAxisSign = 1;
        break;
      }
      case 2:
      {
        xAxisSign = -1;
        yAxisSign = 1;
        break;
      }
      case 4:
      {
        xAxisSign = -1;
        yAxisSign = -1;
        break;
      }
      case 6:
      {
        xAxisSign = +1;
        yAxisSign = -1;
        break;
      }
      // edges
      case 1:
      {
        xAxisSign = 0;
        yAxisSign = 1;
        break;
      }
      case 3:
      {
        xAxisSign = -1;
        yAxisSign = 0;
        break;
      }
      case 5:
      {
        xAxisSign = 0;
        yAxisSign = -1;
        break;
      }
      case 7:
      {
        xAxisSign = 1;
        yAxisSign = 0;
        break;
      }
      default:
        break;
    }

    // if the mouse is being dragged, calculate a new size and also position
    // for resizing the box

    double xMoved = _grabber->GetMouseDownX() - xPos;
    double yMoved = _grabber->GetMouseDownY() - yPos;

    double newWidth = dPtr->width + (xAxisSign * xMoved);
    if (newWidth < 20)
      newWidth  = 20;

    double newHeight = dPtr->height + (yAxisSign * yMoved);
    if (newHeight < 20)
      newHeight = 20;

    double deltaWidth = newWidth - dPtr->width;
    double deltaHeight = newHeight - dPtr->height;

    this->AdjustSize(deltaWidth, deltaHeight);

    deltaWidth *= (-1);
    deltaHeight *= (-1);

    double angle = dPtr->rotationAngle / 360.0 * (2 * M_PI);
    double dx = 0;
    double dy = 0;
    switch (_grabber->GetIndex())
    {
      // grabbers
      case 0:
      {
        dx = sin(-angle) * deltaHeight/2;
        dy = cos(-angle) * deltaHeight/2;
        dx += cos(angle) * deltaWidth/2;
        dy += sin(angle) * deltaWidth/2;
        this->SetPosition(this->pos() + QPointF(dx, dy));
        break;
      }
      case 2:
      {
        dx = sin(-angle) * deltaHeight/2;
        dy = cos(-angle) * deltaHeight/2;
        dx += -cos(angle) * deltaWidth/2;
        dy += -sin(angle) * deltaWidth/2;
        this->SetPosition(this->pos() + QPointF(dx, dy));
        break;
      }
      case 4:
      {
        dx = -sin(-angle) * deltaHeight/2;
        dy = -cos(-angle) * deltaHeight/2;
        dx += -cos(angle) * deltaWidth/2;
        dy += -sin(angle) * deltaWidth/2;
        this->SetPosition(this->pos() + QPointF(dx, dy));
        break;
      }
      case 6:
      {
        dx = -sin(-angle) * deltaHeight/2;
        dy = -cos(-angle) * deltaHeight/2;
        dx += cos(angle) * deltaWidth/2;
        dy += sin(angle) * deltaWidth/2;
        this->SetPosition(this->pos() + QPointF(dx, dy));
        break;
      }
      // edges
      case 1:
      {
        dx = sin(-angle) * deltaHeight/2;
        dy = cos(-angle) * deltaHeight/2;
        this->SetPosition(this->pos() + QPointF(dx, dy));
        break;
      }
      case 3:
      {
        dx = cos(-angle) * deltaWidth/2;
        dy = -sin(-angle) * deltaWidth/2;
        this->SetPosition(this->pos() - QPointF(dx, dy));
        if (this->parentItem())
        {
          WallSegmentItem *wallItem = dynamic_cast<WallSegmentItem *>(
              this->parentItem());
          if (wallItem)
          {
            if (this->GetAngleOnWall() < 90)
            {
              dPtr->positionOnWall -= deltaWidth /
                  (2*wallItem->line().length());
            }
            else
            {
              dPtr->positionOnWall += deltaWidth /
                  (2*wallItem->line().length());
            }
          }
        }
        break;
      }
      case 5:
      {
        dx = sin(-angle) * deltaHeight/2;
        dy = cos(-angle) * deltaHeight/2;
        this->SetPosition(this->pos() - QPointF(dx, dy));
        break;
      }
      case 7:
      {
        dx = cos(angle) * deltaWidth/2;
        dy = sin(angle) * deltaWidth/2;
        this->SetPosition(this->pos() + QPointF(dx, dy));
        if (this->parentItem())
        {
          WallSegmentItem *wallItem = dynamic_cast<WallSegmentItem *>(
              this->parentItem());
          if (wallItem)
          {
            if (this->GetAngleOnWall() < 90)
            {
              dPtr->positionOnWall += deltaWidth /
                  (2*wallItem->line().length());
            }
            else
            {
              dPtr->positionOnWall -= deltaWidth /
                  (2*wallItem->line().length());
            }
          }
        }
        break;
      }
      default:
        break;
    }
    this->UpdateCornerPositions();
    this->update();
  }
  return true;
}

/////////////////////////////////////////////////
void RectItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *_event)
{
  auto dPtr = static_cast<RectItemPrivate *>(this->dataPtr);

  /// TODO: uncomment to enable snap to grid
/*  dPtr->location.setX( (static_cast<int>(dPtr->location.x())
      / dPtr->gridSpace) * dPtr->gridSpace);
  dPtr->location.setY( (static_cast<int>(dPtr->location.y())
      / dPtr->gridSpace) * dPtr->gridSpace);*/

  dPtr->mousePressPos = QPointF(0, 0);
  _event->setAccepted(true);
}

/////////////////////////////////////////////////
void RectItem::mousePressEvent(QGraphicsSceneMouseEvent *_event)
{
  auto dPtr = static_cast<RectItemPrivate *>(this->dataPtr);

//  if (!this->isSelected())
//    this->scene()->clearSelection();

//  this->setSelected(true);
  QApplication::setOverrideCursor(QCursor(Qt::SizeAllCursor));
  dPtr->mousePressPos = this->mapFromScene(_event->scenePos());
  _event->setAccepted(true);
}

/////////////////////////////////////////////////
void RectItem::mouseMoveEvent(QGraphicsSceneMouseEvent *_event)
{
  auto dPtr = static_cast<RectItemPrivate *>(this->dataPtr);

  if (!this->isSelected())
    return;

//  QPointF delta = _event->scenePos() - _event->lastScenePos();
//  this->SetPosition(this->scenePos() + delta);
//  dPtr->location += delta;
//  this->SetPosition(dPtr->location);

  // keep track of mouse press pos for more accurate mouse movements than
  // purely relying on mouse translations because we expect items to rotate
  // arbitrary (snap to parent items) when dragged
  QPointF trans = this->mapFromScene(_event->scenePos()) - dPtr->mousePressPos;
  QPointF rotatedTrans;
  rotatedTrans.setX(cos(GZ_DTOR(dPtr->rotationAngle))*-trans.x()
    - sin(GZ_DTOR(dPtr->rotationAngle))*-trans.y());
  rotatedTrans.setY(sin(GZ_DTOR(dPtr->rotationAngle))*-trans.x()
    + cos(GZ_DTOR(dPtr->rotationAngle))*-trans.y());

  this->SetPosition(this->pos() - rotatedTrans);
}

/////////////////////////////////////////////////
void RectItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *_event)
{
  _event->setAccepted(true);
}

/////////////////////////////////////////////////
void RectItem::hoverLeaveEvent(QGraphicsSceneHoverEvent *_event)
{
  auto dPtr = static_cast<RectItemPrivate *>(this->dataPtr);

  if (!this->isSelected())
  {
    _event->ignore();
    return;
  }

  QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));

  for (int i = 0; i < 8; ++i)
  {
    if (dPtr->grabbers[i]->isEnabled())
      dPtr->grabbers[i]->removeSceneEventFilter(this);
  }
  dPtr->rotateHandle->removeSceneEventFilter(this);
}

/////////////////////////////////////////////////
void RectItem::hoverMoveEvent(QGraphicsSceneHoverEvent *_event)
{
  if (!this->isSelected())
  {
    _event->ignore();
    return;
  }

  QApplication::setOverrideCursor(QCursor(Qt::SizeAllCursor));
}

/////////////////////////////////////////////////
void RectItem::hoverEnterEvent(QGraphicsSceneHoverEvent *_event)
{
  auto dPtr = static_cast<RectItemPrivate *>(this->dataPtr);

  if (!this->isSelected())
  {
    _event->ignore();
    return;
  }

  QApplication::setOverrideCursor(QCursor(Qt::SizeAllCursor));

  for (unsigned int i = 0; i < dPtr->grabbers.size(); ++i)
  {
    if (dPtr->grabbers[i]->isEnabled())
      dPtr->grabbers[i]->installSceneEventFilter(this);
  }
  dPtr->rotateHandle->installSceneEventFilter(this);
}

/////////////////////////////////////////////////
void RectItem::UpdateCornerPositions()
{
  auto dPtr = static_cast<RectItemPrivate *>(this->dataPtr);

  int grabberWidth = (dPtr->grabbers[0]->boundingRect().width())/2;
  int grabberHeight = (dPtr->grabbers[0]->boundingRect().height())/2;

  dPtr->grabbers[0]->setPos(
      dPtr->drawingOriginX - dPtr->drawingWidth/2 - grabberWidth,
      dPtr->drawingOriginY - dPtr->drawingHeight/2 - grabberHeight);
  dPtr->grabbers[2]->setPos(
      dPtr->drawingOriginX + dPtr->drawingWidth/2 - grabberWidth,
      dPtr->drawingOriginY - dPtr->drawingHeight/2 - grabberHeight);
  dPtr->grabbers[4]->setPos(
      dPtr->drawingOriginX + dPtr->drawingWidth/2 - grabberWidth,
      dPtr->drawingOriginY + dPtr->drawingHeight/2 - grabberHeight);
  dPtr->grabbers[6]->setPos(
      dPtr->drawingOriginX - dPtr->drawingWidth/2 - grabberWidth,
      dPtr->drawingOriginY + dPtr->drawingHeight/2 - grabberHeight);

  dPtr->grabbers[1]->setPos(dPtr->drawingOriginX - grabberWidth,
      dPtr->drawingOriginY - dPtr->drawingHeight/2 - grabberHeight);
  dPtr->grabbers[3]->setPos(
      dPtr->drawingOriginX + dPtr->drawingWidth/2 - grabberWidth,
      dPtr->drawingOriginY - grabberHeight);
  dPtr->grabbers[5]->setPos(dPtr->drawingOriginX - grabberWidth,
      dPtr->drawingOriginY + dPtr->drawingHeight/2 - grabberHeight);
  dPtr->grabbers[7]->setPos(
      dPtr->drawingOriginX - dPtr->drawingWidth/2 - grabberWidth,
      dPtr->drawingOriginY - grabberHeight);

  dPtr->rotateHandle->setPos(dPtr->drawingOriginX,
      dPtr->drawingOriginY - dPtr->drawingHeight/2);

  this->SizeChanged();
  this->setRect(this->boundingRect());

//  this->setPolygon(QPolygonF(this->boundingRect()));
}

/////////////////////////////////////////////////
void RectItem::SetWidth(int _width)
{
  auto dPtr = static_cast<RectItemPrivate *>(this->dataPtr);

  dPtr->width = _width;
  dPtr->drawingWidth = dPtr->width;
  this->UpdateCornerPositions();
  this->update();

  emit WidthChanged(dPtr->drawingWidth);
}

/////////////////////////////////////////////////
void RectItem::SetHeight(int _height)
{
  auto dPtr = static_cast<RectItemPrivate *>(this->dataPtr);

  dPtr->height = _height;
  dPtr->drawingHeight = dPtr->height;
  this->UpdateCornerPositions();
  this->update();

  emit DepthChanged(dPtr->drawingHeight);
}

/////////////////////////////////////////////////
void RectItem::SetSize(QSize _size)
{
  auto dPtr = static_cast<RectItemPrivate *>(this->dataPtr);

  dPtr->width = _size.width();
  dPtr->drawingWidth = dPtr->width;
  dPtr->height = _size.height();
  dPtr->drawingHeight = dPtr->height;
  this->UpdateCornerPositions();
  this->update();

  emit WidthChanged(dPtr->drawingWidth);
  emit DepthChanged(dPtr->drawingHeight);
}

/////////////////////////////////////////////////
double RectItem::GetWidth() const
{
  auto dPtr = static_cast<RectItemPrivate *>(this->dataPtr);

  return dPtr->drawingWidth;
}

/////////////////////////////////////////////////
double RectItem::GetHeight() const
{
  auto dPtr = static_cast<RectItemPrivate *>(this->dataPtr);

  return dPtr->drawingHeight;
}

/////////////////////////////////////////////////
void RectItem::SetPositionOnWall(double _positionOnWall)
{
  auto dPtr = static_cast<RectItemPrivate *>(this->dataPtr);

  dPtr->positionOnWall = _positionOnWall;
  this->UpdateMeasures();
}

/////////////////////////////////////////////////
double RectItem::GetPositionOnWall() const
{
  auto dPtr = static_cast<RectItemPrivate *>(this->dataPtr);

  return dPtr->positionOnWall;
}

/////////////////////////////////////////////////
void RectItem::SetAngleOnWall(double _angleOnWall)
{
  auto dPtr = static_cast<RectItemPrivate *>(this->dataPtr);

  dPtr->angleOnWall = _angleOnWall;
  this->UpdateMeasures();
}

/////////////////////////////////////////////////
double RectItem::GetAngleOnWall() const
{
  auto dPtr = static_cast<RectItemPrivate *>(this->dataPtr);

  return dPtr->angleOnWall;
}

/////////////////////////////////////////////////
QRectF RectItem::boundingRect() const
{
  auto dPtr = static_cast<RectItemPrivate *>(this->dataPtr);

  return QRectF(-dPtr->width/2, -dPtr->height/2, dPtr->width, dPtr->height);
}

/////////////////////////////////////////////////
void RectItem::DrawBoundingBox(QPainter *_painter)
{
  _painter->save();
  QPen boundingBoxPen;
  boundingBoxPen.setStyle(Qt::DashDotLine);
  boundingBoxPen.setColor(Qt::darkGray);
  boundingBoxPen.setCapStyle(Qt::RoundCap);
  boundingBoxPen.setJoinStyle(Qt::RoundJoin);
  _painter->setPen(boundingBoxPen);
  _painter->setOpacity(0.8);
  _painter->drawRect(this->boundingRect());
  _painter->restore();
}

/////////////////////////////////////////////////
QVector3D RectItem::GetSize() const
{
  auto dPtr = static_cast<RectItemPrivate *>(this->dataPtr);

  return QVector3D(dPtr->width, dPtr->height, 0);
}

/////////////////////////////////////////////////
QVector3D RectItem::GetScenePosition() const
{
  return QVector3D(this->scenePos().x(), this->scenePos().y(), 0);
}

/////////////////////////////////////////////////
double RectItem::GetSceneRotation() const
{
  auto dPtr = static_cast<RectItemPrivate *>(this->dataPtr);

  return dPtr->rotationAngle;
}

/////////////////////////////////////////////////
void RectItem::paint(QPainter *_painter, const QStyleOptionGraphicsItem *,
    QWidget *)
{
  auto dPtr = static_cast<RectItemPrivate *>(this->dataPtr);

  _painter->save();

  QPointF topLeft(dPtr->drawingOriginX - dPtr->drawingWidth/2,
      dPtr->drawingOriginY - dPtr->drawingHeight/2);
  QPointF topRight(dPtr->drawingOriginX + dPtr->drawingWidth/2,
      dPtr->drawingOriginY - dPtr->drawingHeight/2);
  QPointF bottomLeft(dPtr->drawingOriginX - dPtr->drawingWidth/2,
      dPtr->drawingOriginY + dPtr->drawingHeight/2);
  QPointF bottomRight(dPtr->drawingOriginX  + dPtr->drawingWidth/2,
      dPtr->drawingOriginY + dPtr->drawingHeight/2);

  QPen rectPen;
  rectPen.setStyle(Qt::SolidLine);
  rectPen.setColor(dPtr->borderColor);
  _painter->setPen(rectPen);

  _painter->drawLine(topLeft, topRight);
  _painter->drawLine(topRight, bottomRight);
  _painter->drawLine(bottomRight, bottomLeft);
  _painter->drawLine(bottomLeft, topLeft);
  _painter->restore();
}

/////////////////////////////////////////////////
void RectItem::mouseMoveEvent(QGraphicsSceneDragDropEvent *_event)
{
  _event->setAccepted(false);
}

/////////////////////////////////////////////////
void RectItem::mousePressEvent(QGraphicsSceneDragDropEvent *_event)
{
  _event->setAccepted(false);
}

/////////////////////////////////////////////////
void RectItem::contextMenuEvent(QGraphicsSceneContextMenuEvent *_event)
{
  auto dPtr = static_cast<RectItemPrivate *>(this->dataPtr);

  QMenu menu;
  menu.addAction(dPtr->openInspectorAct);
  menu.addAction(dPtr->deleteItemAct);
  menu.exec(_event->screenPos());
  _event->accept();
}

/////////////////////////////////////////////////
void RectItem::OnOpenInspector()
{
}

/////////////////////////////////////////////////
void RectItem::OnDeleteItem()
{
}

/////////////////////////////////////////////////
void RectItem::SetPosition(const QPointF &_pos)
{
  this->SetPosition(_pos.x(), _pos.y());
}

/////////////////////////////////////////////////
void RectItem::SetPosition(double _x, double _y)
{
  this->setPos(_x, _y);
//  emit posXChanged(_x);
//  emit posYChanged(_y);
}

/////////////////////////////////////////////////
void RectItem::SetRotation(double _angle)
{
  auto dPtr = static_cast<RectItemPrivate *>(this->dataPtr);

  this->rotate(_angle - dPtr->rotationAngle);
  dPtr->rotationAngle = _angle;
  emit YawChanged(dPtr->rotationAngle);
}

/////////////////////////////////////////////////
double RectItem::GetRotation() const
{
  auto dPtr = static_cast<RectItemPrivate *>(this->dataPtr);

  return dPtr->rotationAngle;
}

/////////////////////////////////////////////////
void RectItem::SizeChanged()
{
  auto dPtr = static_cast<RectItemPrivate *>(this->dataPtr);

  emit DepthChanged(dPtr->drawingHeight);
  emit WidthChanged(dPtr->drawingWidth);
  this->UpdateMeasures();
}

/////////////////////////////////////////////////
void RectItem::SetResizeFlag(unsigned int _flag)
{
  auto dPtr = static_cast<RectItemPrivate *>(this->dataPtr);

  if (dPtr->resizeFlag == _flag)
    return;

  dPtr->resizeFlag = _flag;
  for (int i = 0; i < 8; ++i)
    dPtr->grabbers[i]->setEnabled(false);

  if (dPtr->resizeFlag & ITEM_WIDTH)
  {
    dPtr->grabbers[3]->setEnabled(true);
    dPtr->grabbers[7]->setEnabled(true);
  }
  if (dPtr->resizeFlag & ITEM_HEIGHT)
  {
    dPtr->grabbers[1]->setEnabled(true);
    dPtr->grabbers[5]->setEnabled(true);
  }
  if ((dPtr->resizeFlag & ITEM_WIDTH) && (dPtr->resizeFlag & ITEM_HEIGHT))
  {
    dPtr->grabbers[0]->setEnabled(true);
    dPtr->grabbers[2]->setEnabled(true);
    dPtr->grabbers[4]->setEnabled(true);
    dPtr->grabbers[6]->setEnabled(true);
  }
}

/////////////////////////////////////////////////
void RectItem::UpdateMeasures()
{
  auto dPtr = static_cast<RectItemPrivate *>(this->dataPtr);

  // Only windows and doors can have a wall as parent
  WallSegmentItem *wallItem = dynamic_cast<WallSegmentItem *>(
    this->parentItem());
  if (wallItem == NULL)
  {
    for (unsigned int i = 0; i < dPtr->measures.size(); ++i)
    {
      dPtr->measures[i]->setVisible(false);
    }
    return;
  }

  if (dPtr->measures.empty())
  {
    dPtr->measures.push_back(new MeasureItem(QPointF(0, 0), QPointF(0, 1)));
    dPtr->measures.push_back(new MeasureItem(QPointF(0, 0), QPointF(0, 1)));
    dPtr->measures[0]->setVisible(false);
    dPtr->measures[1]->setVisible(false);
  }

  dPtr->measures[0]->setParentItem(wallItem);
  dPtr->measures[1]->setParentItem(wallItem);
  dPtr->measures[0]->setVisible(dPtr->highlighted);
  dPtr->measures[1]->setVisible(dPtr->highlighted);

  if (dPtr->highlighted)
  {
    // Half wall thickness
    double t = wallItem->GetThickness()/2;
    // Distance in px between wall line and measure line
    double d = 20 + t;
    // Half the RectItem's length
    double w = dPtr->drawingWidth/2;
    // This item's angle on the scene
    double angle = GZ_DTOR(dPtr->rotationAngle);
    // Free vector of t on wall direction, for the extremes
    QPointF tVec(t*qCos(angle), t*qSin(angle));
    // Free vector of d perpendicular to the wall
    QPointF dVec(d*qCos(angle+M_PI/2.0), d*qSin(angle+M_PI/2.0));

    QPointF p1wall = wallItem->GetStartPoint();
    QPointF p2wall = wallItem->GetEndPoint();
    QPointF p1rect(this->scenePos().x()-w*qCos(angle),
                   this->scenePos().y()-w*qSin(angle));
    QPointF p2rect(this->scenePos().x()+w*qCos(angle),
                   this->scenePos().y()+w*qSin(angle));

    QPointF extreme1 = p1wall;
    QPointF extreme2 = p2wall;

    // Swap extremes if item is flipped on wall
    if (this->GetAngleOnWall() > 90)
    {
      extreme1 = p2wall;
      extreme2 = p1wall;
    }

    // Measure 0, from extreme 1 to RectItem's start point
    dPtr->measures[0]->SetStartPoint(extreme1 - tVec - dVec);
    dPtr->measures[0]->SetEndPoint(p1rect - dVec);
    // Measure 1, from RectItem's end point to extreme 2
    dPtr->measures[1]->SetStartPoint(p2rect - dVec);
    dPtr->measures[1]->SetEndPoint(extreme2 + tVec - dVec);

    dPtr->measures[0]->SetValue(
        (dPtr->measures[0]->line().length())*dPtr->itemScale);
    dPtr->measures[1]->SetValue(
        (dPtr->measures[1]->line().length())*dPtr->itemScale);
  }
}
