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

#include <ignition/math/Angle.hh>

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
RectItem::RectItem()
    : EditorItem(*new RectItemPrivate),
      rectDPtr(std::static_pointer_cast<RectItemPrivate>(this->editorDPtr))
{
  this->Init();
}

//////////////////////////////////////////////////
RectItem::RectItem(RectItemPrivate &_rectDPtr)
    : EditorItem(_rectDPtr),
      rectDPtr(std::static_pointer_cast<RectItemPrivate>(this->editorDPtr))
{
  this->Init();
}

//////////////////////////////////////////////////
void RectItem::Init()
{
  this->rectDPtr->editorType = "Rect";

  this->rectDPtr->width = 100;
  this->rectDPtr->height = 100;
  this->rectDPtr->highlighted = true;

  this->rectDPtr->drawingOriginX = 0;
  this->rectDPtr->drawingOriginY = 0;

  this->rectDPtr->positionOnWall = 0;
  this->rectDPtr->angleOnWall = 0;

  this->rectDPtr->drawingWidth = this->rectDPtr->width;
  this->rectDPtr->drawingHeight = this->rectDPtr->height;

  this->rectDPtr->borderColor = Qt::black;

  for (int i = 0; i < 8; ++i)
  {
    GrabberHandle *grabber = new GrabberHandle(this, i);
    this->rectDPtr->grabbers.push_back(grabber);
  }
  this->rectDPtr->rotateHandle = new RotateHandle(this);

  this->setSelected(false);
  this->setFlags(this->flags() | QGraphicsItem::ItemIsSelectable);
  this->setFlag(QGraphicsItem::ItemSendsScenePositionChanges);

  this->UpdateCornerPositions();
  this->setAcceptHoverEvents(true);

  this->rectDPtr->cursors.push_back(Qt::SizeFDiagCursor);
  this->rectDPtr->cursors.push_back(Qt::SizeVerCursor);
  this->rectDPtr->cursors.push_back(Qt::SizeBDiagCursor);
  this->rectDPtr->cursors.push_back(Qt::SizeHorCursor);

  this->setCursor(Qt::SizeAllCursor);

  this->rectDPtr->rotationAngle = 0;

  this->rectDPtr->zValueIdle = 1;
  this->rectDPtr->zValueSelected = 5;

  this->SetResizeFlag(ITEM_WIDTH | ITEM_HEIGHT);

  this->rectDPtr->openInspectorAct = new QAction(tr("&Open Inspector"), this);
  this->rectDPtr->openInspectorAct->setStatusTip(tr("Open Inspector"));
  connect(this->rectDPtr->openInspectorAct, SIGNAL(triggered()),
    this, SLOT(OnOpenInspector()));

  this->rectDPtr->deleteItemAct = new QAction(tr("&Delete"), this);
  this->rectDPtr->deleteItemAct->setStatusTip(tr("Delete"));
  connect(this->rectDPtr->deleteItemAct, SIGNAL(triggered()),
    this, SLOT(OnDeleteItem()));
}

/////////////////////////////////////////////////
RectItem::~RectItem()
{
  for (int i = 0; i < 8; ++i)
  {
    this->rectDPtr->grabbers[i]->setParentItem(NULL);
    delete this->rectDPtr->grabbers[i];
  }
  this->rectDPtr->rotateHandle->setParentItem(NULL);
  delete this->rectDPtr->rotateHandle;
  if (!this->rectDPtr->measures.empty())
  {
    delete this->rectDPtr->measures[0];
    delete this->rectDPtr->measures[1];
  }
}

/////////////////////////////////////////////////
void RectItem::ShowHandles(bool _show)
{
  for (int i = 0; i < 8; ++i)
  {
    this->rectDPtr->grabbers[i]->setVisible(_show &&
        this->rectDPtr->grabbers[i]->isEnabled());
  }
  this->rectDPtr->rotateHandle->setVisible(_show);
}

/////////////////////////////////////////////////
void RectItem::AdjustSize(double _x, double _y)
{
  this->rectDPtr->width += _x;
  this->rectDPtr->height += _y;
  this->rectDPtr->drawingWidth = this->rectDPtr->width;
  this->rectDPtr->drawingHeight = this->rectDPtr->height;
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
  if (_highlighted)
  {
    this->setZValue(this->ZValueSelected());
    WallSegmentItem *wallItem = dynamic_cast<WallSegmentItem *>(
        this->parentItem());
    if (wallItem)
      wallItem->setZValue(wallItem->ZValueSelected());

    for (int i = 0; i < 8; ++i)
    {
      if (this->rectDPtr->grabbers[i]->isEnabled())
        this->rectDPtr->grabbers[i]->installSceneEventFilter(this);
    }
    this->rectDPtr->rotateHandle->installSceneEventFilter(this);
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
      if (this->rectDPtr->grabbers[i]->isEnabled())
        this->rectDPtr->grabbers[i]->removeSceneEventFilter(this);
    }
    this->rectDPtr->rotateHandle->removeSceneEventFilter(this);
    this->Set3dTransparency(0.4);
  }
  this->rectDPtr->highlighted = _highlighted;
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
    QPoint localCenter(this->rectDPtr->drawingOriginX,
        this->rectDPtr->drawingOriginY);
    QPointF center = this->mapToScene(localCenter);

    QPointF newPoint = mouseEvent->scenePos();
    QLineF line(center.x(), center.y(), newPoint.x(), newPoint.y());

    double angle = 0;

    if (this->parentItem())
    {
      QPointF localCenterTop(this->rectDPtr->drawingOriginX,
          this->rectDPtr->drawingOriginY
          + this->rectDPtr->drawingHeight);
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
      double angle = this->rectDPtr->rotationAngle
          - static_cast<int>(this->rectDPtr->rotationAngle/360) * 360;
      double range = 22.5;
      if (angle < 0)
        angle += 360;

      if ((angle > (360 - range)) || (angle < range)
          || ((angle <= (180 + range)) && (angle > (180 - range))))
      {
        QApplication::setOverrideCursor(
            QCursor(this->rectDPtr->cursors[_grabber->GetIndex() % 4]));
      }
      else if (((angle <= (360 - range)) && (angle > (270 + range)))
          || ((angle <= (180 - range)) && (angle > (90 + range))))
      {
        QApplication::setOverrideCursor(
            QCursor(this->rectDPtr->cursors[(_grabber->GetIndex() + 3) % 4]));
      }
      else if (((angle <= (270 + range)) && (angle > (270 - range)))
          || ((angle <= (90 + range)) && (angle > (90 - range))))
      {
        QApplication::setOverrideCursor(
            QCursor(this->rectDPtr->cursors[(_grabber->GetIndex() + 2) % 4]));
      }
      else
      {
        QApplication::setOverrideCursor(
            QCursor(this->rectDPtr->cursors[(_grabber->GetIndex() + 1) % 4]));
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

    double newWidth = this->rectDPtr->width + (xAxisSign * xMoved);
    if (newWidth < 20)
      newWidth  = 20;

    double newHeight = this->rectDPtr->height + (yAxisSign * yMoved);
    if (newHeight < 20)
      newHeight = 20;

    double deltaWidth = newWidth - this->rectDPtr->width;
    double deltaHeight = newHeight - this->rectDPtr->height;

    this->AdjustSize(deltaWidth, deltaHeight);

    deltaWidth *= (-1);
    deltaHeight *= (-1);

    double angle = this->rectDPtr->rotationAngle / 360.0 * (2 * M_PI);
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
              this->rectDPtr->positionOnWall -= deltaWidth /
                  (2*wallItem->line().length());
            }
            else
            {
              this->rectDPtr->positionOnWall += deltaWidth /
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
              this->rectDPtr->positionOnWall += deltaWidth /
                  (2*wallItem->line().length());
            }
            else
            {
              this->rectDPtr->positionOnWall -= deltaWidth /
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
  /// TODO: uncomment to enable snap to grid
/*  this->location.setX( (static_cast<int>(this->location.x())
      / this->gridSpace) * this->gridSpace);
  this->location.setY( (static_cast<int>(this->location.y())
      / this->gridSpace) * this->gridSpace);*/

  this->rectDPtr->mousePressPos = QPointF(0, 0);
  _event->setAccepted(true);
}

/////////////////////////////////////////////////
void RectItem::mousePressEvent(QGraphicsSceneMouseEvent *_event)
{
//  if (!this->isSelected())
//    this->scene()->clearSelection();

//  this->setSelected(true);
  QApplication::setOverrideCursor(QCursor(Qt::SizeAllCursor));
  this->rectDPtr->mousePressPos = this->mapFromScene(_event->scenePos());
  _event->setAccepted(true);
}

/////////////////////////////////////////////////
void RectItem::mouseMoveEvent(QGraphicsSceneMouseEvent *_event)
{
  if (!this->isSelected())
    return;

//  QPointF delta = _event->scenePos() - _event->lastScenePos();
//  this->SetPosition(this->scenePos() + delta);
//  this->rectDPtr->location += delta;
//  this->SetPosition(this->rectDPtr->location);

  // keep track of mouse press pos for more accurate mouse movements than
  // purely relying on mouse translations because we expect items to rotate
  // arbitrary (snap to parent items) when dragged
  QPointF trans = this->mapFromScene(_event->scenePos()) -
      this->rectDPtr->mousePressPos;
  QPointF rotatedTrans;
  rotatedTrans.setX(cos(IGN_DTOR(this->rectDPtr->rotationAngle))*-trans.x()
    - sin(IGN_DTOR(this->rectDPtr->rotationAngle))*-trans.y());
  rotatedTrans.setY(sin(IGN_DTOR(this->rectDPtr->rotationAngle))*-trans.x()
    + cos(IGN_DTOR(this->rectDPtr->rotationAngle))*-trans.y());

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
  if (!this->isSelected())
  {
    _event->ignore();
    return;
  }

  QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));

  for (int i = 0; i < 8; ++i)
  {
    if (this->rectDPtr->grabbers[i]->isEnabled())
      this->rectDPtr->grabbers[i]->removeSceneEventFilter(this);
  }
  this->rectDPtr->rotateHandle->removeSceneEventFilter(this);
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
  if (!this->isSelected())
  {
    _event->ignore();
    return;
  }

  QApplication::setOverrideCursor(QCursor(Qt::SizeAllCursor));

  for (unsigned int i = 0; i < this->rectDPtr->grabbers.size(); ++i)
  {
    if (this->rectDPtr->grabbers[i]->isEnabled())
      this->rectDPtr->grabbers[i]->installSceneEventFilter(this);
  }
  this->rectDPtr->rotateHandle->installSceneEventFilter(this);
}

/////////////////////////////////////////////////
void RectItem::UpdateCornerPositions()
{
  int grabberWidth = (this->rectDPtr->grabbers[0]->boundingRect().width())/2;
  int grabberHeight = (this->rectDPtr->grabbers[0]->boundingRect().height())/2;

  this->rectDPtr->grabbers[0]->setPos(
      this->rectDPtr->drawingOriginX - this->rectDPtr->drawingWidth/2 -
      grabberWidth,
      this->rectDPtr->drawingOriginY - this->rectDPtr->drawingHeight/2 -
      grabberHeight);
  this->rectDPtr->grabbers[2]->setPos(
      this->rectDPtr->drawingOriginX + this->rectDPtr->drawingWidth/2 -
      grabberWidth,
      this->rectDPtr->drawingOriginY - this->rectDPtr->drawingHeight/2 -
      grabberHeight);
  this->rectDPtr->grabbers[4]->setPos(
      this->rectDPtr->drawingOriginX + this->rectDPtr->drawingWidth/2 -
      grabberWidth,
      this->rectDPtr->drawingOriginY + this->rectDPtr->drawingHeight/2 -
      grabberHeight);
  this->rectDPtr->grabbers[6]->setPos(
      this->rectDPtr->drawingOriginX - this->rectDPtr->drawingWidth/2 -
      grabberWidth,
      this->rectDPtr->drawingOriginY + this->rectDPtr->drawingHeight/2 -
      grabberHeight);

  this->rectDPtr->grabbers[1]->setPos(this->rectDPtr->drawingOriginX -
      grabberWidth,
      this->rectDPtr->drawingOriginY - this->rectDPtr->drawingHeight/2 -
      grabberHeight);
  this->rectDPtr->grabbers[3]->setPos(
      this->rectDPtr->drawingOriginX + this->rectDPtr->drawingWidth/2 -
      grabberWidth,
      this->rectDPtr->drawingOriginY - grabberHeight);
  this->rectDPtr->grabbers[5]->setPos(this->rectDPtr->drawingOriginX -
      grabberWidth,
      this->rectDPtr->drawingOriginY + this->rectDPtr->drawingHeight/2 -
      grabberHeight);
  this->rectDPtr->grabbers[7]->setPos(
      this->rectDPtr->drawingOriginX - this->rectDPtr->drawingWidth/2 -
      grabberWidth,
      this->rectDPtr->drawingOriginY - grabberHeight);

  this->rectDPtr->rotateHandle->setPos(this->rectDPtr->drawingOriginX,
      this->rectDPtr->drawingOriginY - this->rectDPtr->drawingHeight/2);

  this->SizeChanged();
  this->setRect(this->boundingRect());

//  this->setPolygon(QPolygonF(this->boundingRect()));
}

/////////////////////////////////////////////////
void RectItem::SetWidth(int _width)
{
  this->rectDPtr->width = _width;
  this->rectDPtr->drawingWidth = this->rectDPtr->width;
  this->UpdateCornerPositions();
  this->update();

  emit WidthChanged(this->rectDPtr->drawingWidth);
}

/////////////////////////////////////////////////
void RectItem::SetHeight(int _height)
{
  this->rectDPtr->height = _height;
  this->rectDPtr->drawingHeight = this->rectDPtr->height;
  this->UpdateCornerPositions();
  this->update();

  emit DepthChanged(this->rectDPtr->drawingHeight);
}

/////////////////////////////////////////////////
void RectItem::SetSize(QSize _size)
{
  this->rectDPtr->width = _size.width();
  this->rectDPtr->drawingWidth = this->rectDPtr->width;
  this->rectDPtr->height = _size.height();
  this->rectDPtr->drawingHeight = this->rectDPtr->height;
  this->UpdateCornerPositions();
  this->update();

  emit WidthChanged(this->rectDPtr->drawingWidth);
  emit DepthChanged(this->rectDPtr->drawingHeight);
}

/////////////////////////////////////////////////
double RectItem::GetWidth() const
{
  return this->rectDPtr->drawingWidth;
}

/////////////////////////////////////////////////
double RectItem::GetHeight() const
{
  return this->rectDPtr->drawingHeight;
}

/////////////////////////////////////////////////
void RectItem::SetPositionOnWall(double _positionOnWall)
{
  this->rectDPtr->positionOnWall = _positionOnWall;
  this->UpdateMeasures();
}

/////////////////////////////////////////////////
double RectItem::GetPositionOnWall() const
{
  return this->rectDPtr->positionOnWall;
}

/////////////////////////////////////////////////
void RectItem::SetAngleOnWall(double _angleOnWall)
{
  this->rectDPtr->angleOnWall = _angleOnWall;
  this->UpdateMeasures();
}

/////////////////////////////////////////////////
double RectItem::GetAngleOnWall() const
{
  return this->rectDPtr->angleOnWall;
}

/////////////////////////////////////////////////
QRectF RectItem::boundingRect() const
{
  return QRectF(-this->rectDPtr->width/2, -this->rectDPtr->height/2,
      this->rectDPtr->width, this->rectDPtr->height);
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
  return QVector3D(this->rectDPtr->width, this->rectDPtr->height, 0);
}

/////////////////////////////////////////////////
QVector3D RectItem::GetScenePosition() const
{
  return QVector3D(this->scenePos().x(), this->scenePos().y(), 0);
}

/////////////////////////////////////////////////
double RectItem::GetSceneRotation() const
{
  return this->rectDPtr->rotationAngle;
}

/////////////////////////////////////////////////
void RectItem::paint(QPainter *_painter, const QStyleOptionGraphicsItem *,
    QWidget *)
{
  _painter->save();

  QPointF topLeft(
      this->rectDPtr->drawingOriginX - this->rectDPtr->drawingWidth/2,
      this->rectDPtr->drawingOriginY - this->rectDPtr->drawingHeight/2);
  QPointF topRight(
      this->rectDPtr->drawingOriginX + this->rectDPtr->drawingWidth/2,
      this->rectDPtr->drawingOriginY - this->rectDPtr->drawingHeight/2);
  QPointF bottomLeft(
      this->rectDPtr->drawingOriginX - this->rectDPtr->drawingWidth/2,
      this->rectDPtr->drawingOriginY + this->rectDPtr->drawingHeight/2);
  QPointF bottomRight(
      this->rectDPtr->drawingOriginX + this->rectDPtr->drawingWidth/2,
      this->rectDPtr->drawingOriginY + this->rectDPtr->drawingHeight/2);

  QPen rectPen;
  rectPen.setStyle(Qt::SolidLine);
  rectPen.setColor(this->rectDPtr->borderColor);
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
  QMenu menu;
  menu.addAction(this->rectDPtr->openInspectorAct);
  menu.addAction(this->rectDPtr->deleteItemAct);
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
  this->rotate(_angle - this->rectDPtr->rotationAngle);
  this->rectDPtr->rotationAngle = _angle;
  emit YawChanged(this->rectDPtr->rotationAngle);
}

/////////////////////////////////////////////////
double RectItem::GetRotation() const
{
  return this->rectDPtr->rotationAngle;
}

/////////////////////////////////////////////////
void RectItem::SizeChanged()
{
  emit DepthChanged(this->rectDPtr->drawingHeight);
  emit WidthChanged(this->rectDPtr->drawingWidth);
  this->UpdateMeasures();
}

/////////////////////////////////////////////////
void RectItem::SetResizeFlag(unsigned int _flag)
{
  if (this->rectDPtr->resizeFlag == _flag)
    return;

  this->rectDPtr->resizeFlag = _flag;
  for (int i = 0; i < 8; ++i)
    this->rectDPtr->grabbers[i]->setEnabled(false);

  if (this->rectDPtr->resizeFlag & ITEM_WIDTH)
  {
    this->rectDPtr->grabbers[3]->setEnabled(true);
    this->rectDPtr->grabbers[7]->setEnabled(true);
  }
  if (this->rectDPtr->resizeFlag & ITEM_HEIGHT)
  {
    this->rectDPtr->grabbers[1]->setEnabled(true);
    this->rectDPtr->grabbers[5]->setEnabled(true);
  }
  if ((this->rectDPtr->resizeFlag & ITEM_WIDTH) &&
      (this->rectDPtr->resizeFlag & ITEM_HEIGHT))
  {
    this->rectDPtr->grabbers[0]->setEnabled(true);
    this->rectDPtr->grabbers[2]->setEnabled(true);
    this->rectDPtr->grabbers[4]->setEnabled(true);
    this->rectDPtr->grabbers[6]->setEnabled(true);
  }
}

/////////////////////////////////////////////////
void RectItem::UpdateMeasures()
{
  // Only windows and doors can have a wall as parent
  WallSegmentItem *wallItem = dynamic_cast<WallSegmentItem *>(
    this->parentItem());
  if (wallItem == NULL)
  {
    for (unsigned int i = 0; i < this->rectDPtr->measures.size(); ++i)
    {
      this->rectDPtr->measures[i]->setVisible(false);
    }
    return;
  }

  if (this->rectDPtr->measures.empty())
  {
    this->rectDPtr->measures.push_back(
        new MeasureItem(QPointF(0, 0), QPointF(0, 1)));
    this->rectDPtr->measures.push_back(
        new MeasureItem(QPointF(0, 0), QPointF(0, 1)));
    this->rectDPtr->measures[0]->setVisible(false);
    this->rectDPtr->measures[1]->setVisible(false);
  }

  this->rectDPtr->measures[0]->setParentItem(wallItem);
  this->rectDPtr->measures[1]->setParentItem(wallItem);
  this->rectDPtr->measures[0]->setVisible(this->rectDPtr->highlighted);
  this->rectDPtr->measures[1]->setVisible(this->rectDPtr->highlighted);

  if (this->rectDPtr->highlighted)
  {
    // Half wall thickness
    double t = wallItem->GetThickness()/2;
    // Distance in px between wall line and measure line
    double d = 20 + t;
    // Half the RectItem's length
    double w = this->rectDPtr->drawingWidth/2;
    // This item's angle on the scene
    double angle = IGN_DTOR(this->rectDPtr->rotationAngle);
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
    this->rectDPtr->measures[0]->SetStartPoint(extreme1 - tVec - dVec);
    this->rectDPtr->measures[0]->SetEndPoint(p1rect - dVec);
    // Measure 1, from RectItem's end point to extreme 2
    this->rectDPtr->measures[1]->SetStartPoint(p2rect - dVec);
    this->rectDPtr->measures[1]->SetEndPoint(extreme2 + tVec - dVec);

    this->rectDPtr->measures[0]->SetValue(
        (this->rectDPtr->measures[0]->line().length()) *
         this->rectDPtr->itemScale);
    this->rectDPtr->measures[1]->SetValue(
        (this->rectDPtr->measures[1]->line().length()) *
         this->rectDPtr->itemScale);
  }
}
