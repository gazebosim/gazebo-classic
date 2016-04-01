/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include "gazebo/gui/Conversions.hh"
#include "gazebo/gui/building/BuildingEditorWidget.hh"
#include "gazebo/gui/building/GrabberHandle.hh"
#include "gazebo/gui/building/MeasureItem.hh"
#include "gazebo/gui/building/RectItem.hh"
#include "gazebo/gui/building/RectItemPrivate.hh"
#include "gazebo/gui/building/RotateHandle.hh"
#include "gazebo/gui/building/WallSegmentItem.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
RectItem::RectItem() : EditorItem(), dataPtr(new RectItemPrivate())
{
  this->editorType = "Rect";

  this->width = 100;
  this->height = 100;
  this->highlighted = true;

  this->drawingOriginX = 0;
  this->drawingOriginY = 0;

  this->dataPtr->positionOnWall = 0;
  this->dataPtr->angleOnWall = 0;

  this->drawingWidth = this->width;
  this->drawingHeight = this->height;

  this->borderColor = common::Color::Black;

  for (int i = 0; i < 8; ++i)
  {
    GrabberHandle *grabber = new GrabberHandle(this, i);
    this->dataPtr->grabbers.push_back(grabber);
  }
  this->dataPtr->rotateHandle = new RotateHandle(this);

  this->setSelected(false);
  this->setFlags(this->flags() | QGraphicsItem::ItemIsSelectable);
  this->setFlag(QGraphicsItem::ItemSendsScenePositionChanges);

  this->UpdateCornerPositions();
  this->setAcceptHoverEvents(true);

  this->dataPtr->cursors.push_back(Qt::SizeFDiagCursor);
  this->dataPtr->cursors.push_back(Qt::SizeVerCursor);
  this->dataPtr->cursors.push_back(Qt::SizeBDiagCursor);
  this->dataPtr->cursors.push_back(Qt::SizeHorCursor);

  this->setCursor(Qt::SizeAllCursor);

  this->rotationAngle = 0;

  this->zValueIdle = 1;
  this->zValueSelected = 5;

  this->SetResizeFlag(ITEM_WIDTH | ITEM_HEIGHT);

  this->openInspectorAct = new QAction(tr("&Open Inspector"), this);
  this->openInspectorAct->setStatusTip(tr("Open Inspector"));
  connect(this->openInspectorAct, SIGNAL(triggered()),
    this, SLOT(OnOpenInspector()));

  this->deleteItemAct = new QAction(tr("&Delete"), this);
  this->deleteItemAct->setStatusTip(tr("Delete"));
  connect(this->deleteItemAct, SIGNAL(triggered()),
    this, SLOT(OnDeleteItem()));
}

/////////////////////////////////////////////////
RectItem::~RectItem()
{
  for (int i = 0; i < 8; ++i)
  {
    this->dataPtr->grabbers[i]->setParentItem(NULL);
    delete this->dataPtr->grabbers[i];
  }

  this->dataPtr->rotateHandle->setParentItem(NULL);
  delete this->dataPtr->rotateHandle;

  if (!this->parentItem() && !this->measures.empty())
  {
    delete this->measures[0];
    delete this->measures[1];
  }
}

/////////////////////////////////////////////////
void RectItem::ShowHandles(bool _show)
{
  for (int i = 0; i < 8; ++i)
  {
    this->dataPtr->grabbers[i]->setVisible(_show &&
        this->dataPtr->grabbers[i]->isEnabled());
  }
  this->dataPtr->rotateHandle->setVisible(_show);
}

/////////////////////////////////////////////////
void RectItem::AdjustSize(double _x, double _y)
{
  this->width += _x;
  this->height += _y;
  this->drawingWidth = this->width;
  this->drawingHeight = this->height;
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
      if (this->dataPtr->grabbers[i]->isEnabled())
        this->dataPtr->grabbers[i]->installSceneEventFilter(this);
    }
    this->dataPtr->rotateHandle->installSceneEventFilter(this);
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
      if (this->dataPtr->grabbers[i]->isEnabled())
        this->dataPtr->grabbers[i]->removeSceneEventFilter(this);
    }
    this->dataPtr->rotateHandle->removeSceneEventFilter(this);
    this->Set3dTransparency(0.4);
  }
  this->highlighted = _highlighted;
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

  if (_rotate->MouseState()
      == static_cast<int>(QEvent::GraphicsSceneMouseMove))
  {
    QPoint localCenter(this->drawingOriginX, this->drawingOriginY);
    QPointF center = this->mapToScene(localCenter);

    QPointF newPoint = mouseEvent->scenePos();
    QLineF line(center.x(), center.y(), newPoint.x(), newPoint.y());

    double angle = 0;

    if (this->parentItem())
    {
      QPointF localCenterTop(this->drawingOriginX, this->drawingOriginY
          + this->drawingHeight);
      QPointF centerTop = this->mapToScene(localCenterTop);
      QLineF lineCenter(center.x(), center.y(), centerTop.x(), centerTop.y());
      angle = -lineCenter.angleTo(line);

      if (angle < 0)
        angle += 360;
      if (angle < 90 || angle > 270)
      {
        angle = 180;
        this->SetRotation(this->Rotation() + angle);
        if (this->AngleOnWall() < 90)
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
      this->SetRotation(this->Rotation() + angle);
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
      double angle = this->rotationAngle
          - static_cast<int>(this->rotationAngle/360) * 360;
      double range = 22.5;
      if (angle < 0)
        angle += 360;

      if ((angle > (360 - range)) || (angle < range)
          || ((angle <= (180 + range)) && (angle > (180 - range))))
      {
        QApplication::setOverrideCursor(
            QCursor(this->dataPtr->cursors[_grabber->Index() % 4]));
      }
      else if (((angle <= (360 - range)) && (angle > (270 + range)))
          || ((angle <= (180 - range)) && (angle > (90 + range))))
      {
        QApplication::setOverrideCursor(
            QCursor(this->dataPtr->cursors[(_grabber->Index() + 3) % 4]));
      }
      else if (((angle <= (270 + range)) && (angle > (270 - range)))
          || ((angle <= (90 + range)) && (angle > (90 - range))))
      {
        QApplication::setOverrideCursor(
            QCursor(this->dataPtr->cursors[(_grabber->Index() + 2) % 4]));
      }
      else
      {
        QApplication::setOverrideCursor(
            QCursor(this->dataPtr->cursors[(_grabber->Index() + 1) % 4]));
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


  if (_grabber->MouseState()
      == static_cast<int>(QEvent::GraphicsSceneMouseMove))
  {
    double xPos = mouseEvent->pos().x();
    double yPos = mouseEvent->pos().y();

    // depending on which grabber has been grabbed, we want to move the position
    // of the item as it grows/shrinks accordingly. so we need to either add
    // or subtract the offsets based on which grabber this is.

    int xAxisSign = 0;
    int yAxisSign = 0;
    switch (_grabber->Index())
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

    double xMoved = _grabber->MouseDownX() - xPos;
    double yMoved = _grabber->MouseDownY() - yPos;

    double newWidth = this->width + (xAxisSign * xMoved);
    if (newWidth < 20)
      newWidth  = 20;

    double newHeight = this->height + (yAxisSign * yMoved);
    if (newHeight < 20)
      newHeight = 20;

    double deltaWidth = newWidth - this->width;
    double deltaHeight = newHeight - this->height;

    this->AdjustSize(deltaWidth, deltaHeight);

    deltaWidth *= (-1);
    deltaHeight *= (-1);

    double angle = this->rotationAngle / 360.0 * (2 * M_PI);
    double dx = 0;
    double dy = 0;
    auto currentPos = Conversions::Convert(this->pos());
    switch (_grabber->Index())
    {
      // grabbers
      case 0:
      {
        dx = sin(-angle) * deltaHeight/2;
        dy = cos(-angle) * deltaHeight/2;
        dx += cos(angle) * deltaWidth/2;
        dy += sin(angle) * deltaWidth/2;
        this->SetPosition(currentPos + ignition::math::Vector2d(dx, dy));
        break;
      }
      case 2:
      {
        dx = sin(-angle) * deltaHeight/2;
        dy = cos(-angle) * deltaHeight/2;
        dx += -cos(angle) * deltaWidth/2;
        dy += -sin(angle) * deltaWidth/2;
        this->SetPosition(currentPos + ignition::math::Vector2d(dx, dy));
        break;
      }
      case 4:
      {
        dx = -sin(-angle) * deltaHeight/2;
        dy = -cos(-angle) * deltaHeight/2;
        dx += -cos(angle) * deltaWidth/2;
        dy += -sin(angle) * deltaWidth/2;
        this->SetPosition(currentPos + ignition::math::Vector2d(dx, dy));
        break;
      }
      case 6:
      {
        dx = -sin(-angle) * deltaHeight/2;
        dy = -cos(-angle) * deltaHeight/2;
        dx += cos(angle) * deltaWidth/2;
        dy += sin(angle) * deltaWidth/2;
        this->SetPosition(currentPos + ignition::math::Vector2d(dx, dy));
        break;
      }
      // edges
      case 1:
      {
        dx = sin(-angle) * deltaHeight/2;
        dy = cos(-angle) * deltaHeight/2;
        this->SetPosition(currentPos + ignition::math::Vector2d(dx, dy));
        break;
      }
      case 3:
      {
        dx = cos(-angle) * deltaWidth/2;
        dy = -sin(-angle) * deltaWidth/2;
        this->SetPosition(currentPos - ignition::math::Vector2d(dx, dy));
        if (this->parentItem())
        {
          WallSegmentItem *wallItem = dynamic_cast<WallSegmentItem *>(
              this->parentItem());
          if (wallItem)
          {
            if (this->AngleOnWall() < 90)
            {
              this->dataPtr->positionOnWall -= deltaWidth /
                  (2*wallItem->line().length());
            }
            else
            {
              this->dataPtr->positionOnWall += deltaWidth /
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
        this->SetPosition(currentPos - ignition::math::Vector2d(dx, dy));
        break;
      }
      case 7:
      {
        dx = cos(angle) * deltaWidth/2;
        dy = sin(angle) * deltaWidth/2;
        this->SetPosition(currentPos + ignition::math::Vector2d(dx, dy));
        if (this->parentItem())
        {
          WallSegmentItem *wallItem = dynamic_cast<WallSegmentItem *>(
              this->parentItem());
          if (wallItem)
          {
            if (this->AngleOnWall() < 90)
            {
              this->dataPtr->positionOnWall += deltaWidth /
                  (2*wallItem->line().length());
            }
            else
            {
              this->dataPtr->positionOnWall -= deltaWidth /
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

  this->dataPtr->mousePressPos = ignition::math::Vector2d::Zero;
  _event->setAccepted(true);
}

/////////////////////////////////////////////////
void RectItem::mousePressEvent(QGraphicsSceneMouseEvent *_event)
{
//  if (!this->isSelected())
//    this->scene()->clearSelection();

//  this->setSelected(true);
  QApplication::setOverrideCursor(QCursor(Qt::SizeAllCursor));
  this->dataPtr->mousePressPos =
      Conversions::Convert(this->mapFromScene(_event->scenePos()));
  _event->setAccepted(true);
}

/////////////////////////////////////////////////
void RectItem::mouseMoveEvent(QGraphicsSceneMouseEvent *_event)
{
  if (!this->isSelected())
    return;

//  QPointF delta = _event->scenePos() - _event->lastScenePos();
//  this->SetPosition(this->scenePos() + delta);
//  this->dataPtr->location += delta;
//  this->SetPosition(this->dataPtr->location);

  // keep track of mouse press pos for more accurate mouse movements than
  // purely relying on mouse translations because we expect items to rotate
  // arbitrary (snap to parent items) when dragged
  QPointF trans = this->mapFromScene(_event->scenePos()) -
      Conversions::Convert(this->dataPtr->mousePressPos);
  QPointF rotatedTrans;
  rotatedTrans.setX(cos(IGN_DTOR(this->rotationAngle))*-trans.x()
    - sin(IGN_DTOR(this->rotationAngle))*-trans.y());
  rotatedTrans.setY(sin(IGN_DTOR(this->rotationAngle))*-trans.x()
    + cos(IGN_DTOR(this->rotationAngle))*-trans.y());

  this->SetPosition(Conversions::Convert(this->pos() - rotatedTrans));
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
    if (this->dataPtr->grabbers[i]->isEnabled())
      this->dataPtr->grabbers[i]->removeSceneEventFilter(this);
  }
  this->dataPtr->rotateHandle->removeSceneEventFilter(this);
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

  for (unsigned int i = 0; i < this->dataPtr->grabbers.size(); ++i)
  {
    if (this->dataPtr->grabbers[i]->isEnabled())
      this->dataPtr->grabbers[i]->installSceneEventFilter(this);
  }
  this->dataPtr->rotateHandle->installSceneEventFilter(this);
}

/////////////////////////////////////////////////
void RectItem::UpdateCornerPositions()
{
  int grabberWidth = (this->dataPtr->grabbers[0]->boundingRect().width())/2;
  int grabberHeight = (this->dataPtr->grabbers[0]->boundingRect().height())/2;

  this->dataPtr->grabbers[0]->setPos(
      this->drawingOriginX - this->drawingWidth/2 - grabberWidth,
      this->drawingOriginY - this->drawingHeight/2 - grabberHeight);
  this->dataPtr->grabbers[2]->setPos(
      this->drawingOriginX + this->drawingWidth/2 - grabberWidth,
      this->drawingOriginY - this->drawingHeight/2 - grabberHeight);
  this->dataPtr->grabbers[4]->setPos(
      this->drawingOriginX + this->drawingWidth/2 - grabberWidth,
      this->drawingOriginY + this->drawingHeight/2 - grabberHeight);
  this->dataPtr->grabbers[6]->setPos(
      this->drawingOriginX - this->drawingWidth/2 -grabberWidth,
      this->drawingOriginY + this->drawingHeight/2 - grabberHeight);

  this->dataPtr->grabbers[1]->setPos(
      this->drawingOriginX - grabberWidth,
      this->drawingOriginY - this->drawingHeight/2 - grabberHeight);
  this->dataPtr->grabbers[3]->setPos(
      this->drawingOriginX + this->drawingWidth/2 - grabberWidth,
      this->drawingOriginY - grabberHeight);
  this->dataPtr->grabbers[5]->setPos(
      this->drawingOriginX - grabberWidth,
      this->drawingOriginY + this->drawingHeight/2 - grabberHeight);
  this->dataPtr->grabbers[7]->setPos(
      this->drawingOriginX - this->drawingWidth/2 - grabberWidth,
      this->drawingOriginY - grabberHeight);

  this->dataPtr->rotateHandle->setPos(this->drawingOriginX,
      this->drawingOriginY - this->drawingHeight/2);

  this->SizeChanged();
  this->setRect(this->boundingRect());

//  this->setPolygon(QPolygonF(this->boundingRect()));
}

/////////////////////////////////////////////////
void RectItem::SetWidth(int _width)
{
  this->width = _width;
  this->drawingWidth = this->width;
  this->UpdateCornerPositions();
  this->update();

  emit WidthChanged(this->drawingWidth);
}

/////////////////////////////////////////////////
void RectItem::SetHeight(int _height)
{
  this->height = _height;
  this->drawingHeight = this->height;
  this->UpdateCornerPositions();
  this->update();

  emit DepthChanged(this->drawingHeight);
}

/////////////////////////////////////////////////
void RectItem::SetSize(const ignition::math::Vector2i &_size)
{
  this->width = _size.X();
  this->drawingWidth = this->width;
  this->height = _size.Y();
  this->drawingHeight = this->height;
  this->UpdateCornerPositions();
  this->update();

  emit WidthChanged(this->drawingWidth);
  emit DepthChanged(this->drawingHeight);
}

/////////////////////////////////////////////////
double RectItem::Width() const
{
  return this->drawingWidth;
}

/////////////////////////////////////////////////
double RectItem::Height() const
{
  return this->drawingHeight;
}

/////////////////////////////////////////////////
void RectItem::SetPositionOnWall(double _positionOnWall)
{
  this->dataPtr->positionOnWall = _positionOnWall;
  this->UpdateMeasures();
}

/////////////////////////////////////////////////
double RectItem::PositionOnWall() const
{
  return this->dataPtr->positionOnWall;
}

/////////////////////////////////////////////////
void RectItem::SetAngleOnWall(double _angleOnWall)
{
  this->dataPtr->angleOnWall = _angleOnWall;
  this->UpdateMeasures();
}

/////////////////////////////////////////////////
double RectItem::AngleOnWall() const
{
  return this->dataPtr->angleOnWall;
}

/////////////////////////////////////////////////
QRectF RectItem::boundingRect() const
{
  return QRectF(-this->width/2, -this->height/2, this->width, this->height);
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
ignition::math::Vector3d RectItem::Size() const
{
  return ignition::math::Vector3d(this->width, this->height, 0);
}

/////////////////////////////////////////////////
ignition::math::Vector3d RectItem::ScenePosition() const
{
  return ignition::math::Vector3d(
      this->scenePos().x(), this->scenePos().y(), 0);
}

/////////////////////////////////////////////////
double RectItem::SceneRotation() const
{
  return this->rotationAngle;
}

/////////////////////////////////////////////////
void RectItem::paint(QPainter *_painter, const QStyleOptionGraphicsItem *,
    QWidget *)
{
  _painter->save();

  QPointF topLeft(this->drawingOriginX - this->drawingWidth/2,
      this->drawingOriginY - this->drawingHeight/2);
  QPointF topRight(this->drawingOriginX + this->drawingWidth/2,
      this->drawingOriginY - this->drawingHeight/2);
  QPointF bottomLeft(this->drawingOriginX - this->drawingWidth/2,
      this->drawingOriginY + this->drawingHeight/2);
  QPointF bottomRight(this->drawingOriginX + this->drawingWidth/2,
      this->drawingOriginY + this->drawingHeight/2);

  QPen rectPen;
  rectPen.setStyle(Qt::SolidLine);
  rectPen.setColor(Conversions::Convert(this->borderColor));
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
  menu.addAction(this->openInspectorAct);
  menu.addAction(this->deleteItemAct);
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
void RectItem::SetPosition(const ignition::math::Vector2d &_pos)
{
  this->SetPosition(_pos.X(), _pos.Y());
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
  this->rotate(_angle - this->rotationAngle);
  this->rotationAngle = _angle;
  emit YawChanged(this->rotationAngle);
}

/////////////////////////////////////////////////
double RectItem::Rotation() const
{
  return this->rotationAngle;
}

/////////////////////////////////////////////////
void RectItem::SizeChanged()
{
  emit DepthChanged(this->drawingHeight);
  emit WidthChanged(this->drawingWidth);
  this->UpdateMeasures();
}

/////////////////////////////////////////////////
void RectItem::SetResizeFlag(unsigned int _flag)
{
  if (this->dataPtr->resizeFlag == _flag)
    return;

  this->dataPtr->resizeFlag = _flag;
  for (int i = 0; i < 8; ++i)
    this->dataPtr->grabbers[i]->setEnabled(false);

  if (this->dataPtr->resizeFlag & ITEM_WIDTH)
  {
    this->dataPtr->grabbers[3]->setEnabled(true);
    this->dataPtr->grabbers[7]->setEnabled(true);
  }
  if (this->dataPtr->resizeFlag & ITEM_HEIGHT)
  {
    this->dataPtr->grabbers[1]->setEnabled(true);
    this->dataPtr->grabbers[5]->setEnabled(true);
  }
  if ((this->dataPtr->resizeFlag & ITEM_WIDTH) &&
      (this->dataPtr->resizeFlag & ITEM_HEIGHT))
  {
    this->dataPtr->grabbers[0]->setEnabled(true);
    this->dataPtr->grabbers[2]->setEnabled(true);
    this->dataPtr->grabbers[4]->setEnabled(true);
    this->dataPtr->grabbers[6]->setEnabled(true);
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
    for (unsigned int i = 0; i < this->measures.size(); ++i)
    {
      this->measures[i]->setVisible(false);
    }
    return;
  }

  if (this->measures.empty())
  {
    this->measures.push_back(new MeasureItem(ignition::math::Vector2d(0, 0),
                                             ignition::math::Vector2d(0, 1)));
    this->measures.push_back(new MeasureItem(ignition::math::Vector2d(0, 0),
                                             ignition::math::Vector2d(0, 1)));
    this->measures[0]->setVisible(false);
    this->measures[1]->setVisible(false);
  }

  this->measures[0]->setParentItem(wallItem);
  this->measures[1]->setParentItem(wallItem);
  this->measures[0]->setVisible(this->highlighted);
  this->measures[1]->setVisible(this->highlighted);

  if (this->highlighted)
  {
    // Half wall thickness
    double t = wallItem->Thickness()/2;
    // Distance in px between wall line and measure line
    double d = 20 + t;
    // Half the RectItem's length
    double w = this->drawingWidth/2;
    // This item's angle on the scene
    double angle = IGN_DTOR(this->rotationAngle);
    // Free vector of t on wall direction, for the extremes
    ignition::math::Vector2d tVec(t*qCos(angle), t*qSin(angle));
    // Free vector of d perpendicular to the wall
    ignition::math::Vector2d dVec(d*cos(angle+M_PI/2.0),
                                  d*sin(angle+M_PI/2.0));

    auto p1wall = wallItem->StartPoint();
    auto p2wall = wallItem->EndPoint();
    ignition::math::Vector2d p1rect(this->scenePos().x()-w*qCos(angle),
                                    this->scenePos().y()-w*qSin(angle));
    ignition::math::Vector2d p2rect(this->scenePos().x()+w*qCos(angle),
                                    this->scenePos().y()+w*qSin(angle));

    auto extreme1 = p1wall;
    auto extreme2 = p2wall;

    // Swap extremes if item is flipped on wall
    if (this->AngleOnWall() > 90)
    {
      extreme1 = p2wall;
      extreme2 = p1wall;
    }

    // Measure 0, from extreme 1 to RectItem's start point
    this->measures[0]->SetStartPoint(extreme1 - tVec - dVec);
    this->measures[0]->SetEndPoint(p1rect - dVec);
    // Measure 1, from RectItem's end point to extreme 2
    this->measures[1]->SetStartPoint(p2rect - dVec);
    this->measures[1]->SetEndPoint(extreme2 + tVec - dVec);

    this->measures[0]->SetValue(
        (this->measures[0]->line().length())*this->itemScale);
    this->measures[1]->SetValue(
        (this->measures[1]->line().length())*this->itemScale);
  }
}

/////////////////////////////////////////////////
void RectItem::DetachFromParent()
{
  this->setParentItem(NULL);
  for (unsigned int i = 0; i < this->measures.size(); i++)
    this->measures[i]->setParentItem(NULL);
}
