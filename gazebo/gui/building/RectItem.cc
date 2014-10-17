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
#include "gazebo/gui/building/BuildingEditorWidget.hh"
#include "gazebo/gui/building/GrabberHandle.hh"
#include "gazebo/gui/building/RotateHandle.hh"
#include "gazebo/gui/building/EditorItem.hh"
#include "gazebo/gui/building/RectItem.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
RectItem::RectItem()
{
  this->editorType = "Rect";

  this->width = 100;
  this->height = 100;

  this->drawingOriginX = 0;
  this->drawingOriginY = 0;

  this->positionOnWall = 0;

  this->drawingWidth = this->width;
  this->drawingHeight = this->height;

  this->borderColor = Qt::black;

  for (int i = 0; i < 8; ++i)
  {
    GrabberHandle *grabber = new GrabberHandle(this, i);
    this->grabbers.push_back(grabber);
  }
  this->rotateHandle = new RotateHandle(this);

  this->setSelected(false);
  this->setFlags(this->flags() | QGraphicsItem::ItemIsSelectable);
  this->setFlag(QGraphicsItem::ItemSendsScenePositionChanges);

  this->UpdateCornerPositions();
  this->setAcceptHoverEvents(true);

  this->cursors.push_back(Qt::SizeFDiagCursor);
  this->cursors.push_back(Qt::SizeVerCursor);
  this->cursors.push_back(Qt::SizeBDiagCursor);
  this->cursors.push_back(Qt::SizeHorCursor);

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
    this->grabbers[i]->setParentItem(NULL);
    delete this->grabbers[i];
  }
  this->rotateHandle->setParentItem(NULL);
  delete this->rotateHandle;
}

/////////////////////////////////////////////////
void RectItem::ShowHandles(bool _show)
{
  for (int i = 0; i < 8; ++i)
  {
    this->grabbers[i]->setVisible(_show && this->grabbers[i]->isEnabled());
  }
  this->rotateHandle->setVisible(_show);
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
    if (_value.toBool())
    {
      this->setZValue(zValueSelected);
      for (int i = 0; i < 8; ++i)
      {
        if (this->grabbers[i]->isEnabled())
          this->grabbers[i]->installSceneEventFilter(this);
      }
      this->rotateHandle->installSceneEventFilter(this);
    }
    else
    {
      this->setZValue(zValueIdle);
      for (int i = 0; i < 8; ++i)
      {
        if (this->grabbers[i]->isEnabled())
          this->grabbers[i]->removeSceneEventFilter(this);
      }
      this->rotateHandle->removeSceneEventFilter(this);
    }
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
        this->SetRotation(this->GetRotation() + angle);
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
      double angle = this->rotationAngle
          - static_cast<int>(rotationAngle/360) * 360;
      double range = 22.5;
      if (angle < 0)
        angle += 360;

      if ((angle > (360 - range)) || (angle < range)
          || ((angle <= (180 + range)) && (angle > (180 - range))))
      {
        QApplication::setOverrideCursor(
            QCursor(this->cursors[_grabber->GetIndex() % 4]));
      }
      else if (((angle <= (360 - range)) && (angle > (270 + range)))
          || ((angle <= (180 - range)) && (angle > (90 + range))))
      {
        QApplication::setOverrideCursor(
            QCursor(this->cursors[(_grabber->GetIndex() + 3) % 4]));
      }
      else if (((angle <= (270 + range)) && (angle > (270 - range)))
          || ((angle <= (90 + range)) && (angle > (90 - range))))
      {
        QApplication::setOverrideCursor(
            QCursor(this->cursors[(_grabber->GetIndex() + 2) % 4]));
      }
      else
      {
        QApplication::setOverrideCursor(
            QCursor(this->cursors[(_grabber->GetIndex() + 1) % 4]));
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

    double angle = rotationAngle / 360.0 * (2 * M_PI);
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

  this->mousePressPos = QPointF(0, 0);
  _event->setAccepted(true);
}

/////////////////////////////////////////////////
void RectItem::mousePressEvent(QGraphicsSceneMouseEvent *_event)
{
//  if (!this->isSelected())
//    this->scene()->clearSelection();

//  this->setSelected(true);
  QApplication::setOverrideCursor(QCursor(Qt::SizeAllCursor));
  this->mousePressPos = this->mapFromScene(_event->scenePos());
  _event->setAccepted(true);
}

/////////////////////////////////////////////////
void RectItem::mouseMoveEvent(QGraphicsSceneMouseEvent *_event)
{
  if (!this->isSelected())
    return;

//  QPointF delta = _event->scenePos() - _event->lastScenePos();
//  this->SetPosition(this->scenePos() + delta);
//  this->location += delta;
//  this->SetPosition(this->location);

  // keep track of mouse press pos for more accurate mouse movements than
  // purely relying on mouse translations because we expect items to rotate
  // arbitrary (snap to parent items) when dragged
  QPointF trans = this->mapFromScene(_event->scenePos()) - this->mousePressPos;
  QPointF rotatedTrans;
  rotatedTrans.setX(cos(GZ_DTOR(this->rotationAngle))*-trans.x()
    - sin(GZ_DTOR(this->rotationAngle))*-trans.y());
  rotatedTrans.setY(sin(GZ_DTOR(this->rotationAngle))*-trans.x()
    + cos(GZ_DTOR(this->rotationAngle))*-trans.y());

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
    if (this->grabbers[i]->isEnabled())
      this->grabbers[i]->removeSceneEventFilter(this);
  }
  this->rotateHandle->removeSceneEventFilter(this);
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

  for (unsigned int i = 0; i < this->grabbers.size(); ++i)
  {
    if (this->grabbers[i]->isEnabled())
      this->grabbers[i]->installSceneEventFilter(this);
  }
  this->rotateHandle->installSceneEventFilter(this);
}

/////////////////////////////////////////////////
void RectItem::UpdateCornerPositions()
{
  int grabberWidth = (this->grabbers[0]->boundingRect().width())/2;
  int grabberHeight = (this->grabbers[0]->boundingRect().height())/2;

  this->grabbers[0]->setPos(
      this->drawingOriginX - this->drawingWidth/2 - grabberWidth,
      this->drawingOriginY - this->drawingHeight/2 - grabberHeight);
  this->grabbers[2]->setPos(
      this->drawingOriginX + this->drawingWidth/2 - grabberWidth,
      this->drawingOriginY - this->drawingHeight/2 - grabberHeight);
  this->grabbers[4]->setPos(
      this->drawingOriginX + this->drawingWidth/2 - grabberWidth,
      this->drawingOriginY + this->drawingHeight/2 - grabberHeight);
  this->grabbers[6]->setPos(
      this->drawingOriginX - this->drawingWidth/2 - grabberWidth,
      this->drawingOriginY + this->drawingHeight/2 - grabberHeight);

  this->grabbers[1]->setPos(this->drawingOriginX - grabberWidth,
      this->drawingOriginY - this->drawingHeight/2 - grabberHeight);
  this->grabbers[3]->setPos(
      this->drawingOriginX + this->drawingWidth/2 - grabberWidth,
      this->drawingOriginY - grabberHeight);
  this->grabbers[5]->setPos(this->drawingOriginX - grabberWidth,
      this->drawingOriginY + this->drawingHeight/2 - grabberHeight);
  this->grabbers[7]->setPos(
      this->drawingOriginX - this->drawingWidth/2 - grabberWidth,
      this->drawingOriginY - grabberHeight);

  this->rotateHandle->setPos(this->drawingOriginX,
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
void RectItem::SetSize(QSize _size)
{
  this->width = _size.width();
  this->drawingWidth = this->width;
  this->height = _size.height();
  this->drawingHeight = this->height;
  this->UpdateCornerPositions();
  this->update();

  emit WidthChanged(this->drawingWidth);
  emit DepthChanged(this->drawingHeight);
}

/////////////////////////////////////////////////
double RectItem::GetWidth() const
{
  return this->drawingWidth;
}

/////////////////////////////////////////////////
double RectItem::GetHeight() const
{
  return this->drawingHeight;
}

/////////////////////////////////////////////////
void RectItem::SetPositionOnWall(double _positionOnWall)
{
  this->positionOnWall = _positionOnWall;
}

/////////////////////////////////////////////////
double RectItem::GetPositionOnWall() const
{
  return this->positionOnWall;
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
QVector3D RectItem::GetSize() const
{
  return QVector3D(this->width, this->height, 0);
}

/////////////////////////////////////////////////
QVector3D RectItem::GetScenePosition() const
{
  return QVector3D(this->scenePos().x(), this->scenePos().y(), 0);
}

/////////////////////////////////////////////////
double RectItem::GetSceneRotation() const
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
  QPointF bottomRight(this->drawingOriginX  + this->drawingWidth/2,
      this->drawingOriginY + this->drawingHeight/2);

  QPen rectPen;
  rectPen.setStyle(Qt::SolidLine);
  rectPen.setColor(borderColor);
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
  this->rotate(_angle - this->rotationAngle);
  this->rotationAngle = _angle;
  emit YawChanged(this->rotationAngle);
}

/////////////////////////////////////////////////
double RectItem::GetRotation() const
{
  return this->rotationAngle;
}

/////////////////////////////////////////////////
void RectItem::SizeChanged()
{
  emit DepthChanged(this->drawingHeight);
  emit WidthChanged(this->drawingWidth);
}

/////////////////////////////////////////////////
void RectItem::SetResizeFlag(unsigned int _flag)
{
  if (this->resizeFlag == _flag)
    return;

  this->resizeFlag = _flag;
  for (int i = 0; i < 8; ++i)
    this->grabbers[i]->setEnabled(false);


  if (resizeFlag & ITEM_WIDTH)
  {
    this->grabbers[3]->setEnabled(true);
    this->grabbers[7]->setEnabled(true);
  }
  if (resizeFlag & ITEM_HEIGHT)
  {
    this->grabbers[1]->setEnabled(true);
    this->grabbers[5]->setEnabled(true);
  }
  if ((resizeFlag & ITEM_WIDTH) && (resizeFlag & ITEM_HEIGHT))
  {
    this->grabbers[0]->setEnabled(true);
    this->grabbers[2]->setEnabled(true);
    this->grabbers[4]->setEnabled(true);
    this->grabbers[6]->setEnabled(true);
  }
}
