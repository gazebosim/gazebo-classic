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

#include "RectItem.hh"
#include "CornerGrabber.hh"
#include "RotateHandle.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
RectItem::RectItem():
    borderColor(Qt::black),
    location(0,0),
    gridSpace(10),
    xCornerGrabBuffer(10),
    yCornerGrabBuffer(10)
{
  this->width = 100;
  this->height = 100;

  this->drawingOriginX = 0;
  this->drawingOriginY = 0;

  this->drawingWidth = this->width;
  this->drawingHeight = this->height;

  for (int i = 0; i < 8; ++i)
    this->corners[i] = new CornerGrabber(this, i);
  this->rotateHandle = new RotateHandle(this);

  this->setSelected(false);
  this->setFlags(this->flags() | QGraphicsItem::ItemIsSelectable);

  this->UpdateCornerPositions();
  this->setAcceptHoverEvents(true);

  QPen transparentPen(QColor(0, 0, 0, 0));
  this->setPen(transparentPen);

  this->rotationAngle = 0;
}

 /////////////////////////////////////////////////
RectItem::~RectItem()
{
  for (int i = 0; i < 8; ++i)
  {
    this->corners[i]->setParentItem(NULL);
    delete this->corners[i];
  }
  this->rotateHandle->setParentItem(NULL);
  delete this->rotateHandle;
}

/////////////////////////////////////////////////
void RectItem::showCorners(bool _show)
{
  for (int i = 0; i < 8; ++i)
    this->corners[i]->setVisible(_show);
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
bool RectItem::sceneEventFilter(QGraphicsItem * _watched, QEvent *_event)
{
  QGraphicsSceneMouseEvent *event =
      dynamic_cast<QGraphicsSceneMouseEvent*>(_event);
  if (event == NULL)
    return false;

  RotateHandle *rotateH = dynamic_cast<RotateHandle *>(_watched);
  if (rotateH != NULL)
    return this->rotateEventFilter(rotateH, event);

  CornerGrabber *corner = dynamic_cast<CornerGrabber *>(_watched);
  if (corner != NULL)
    return this->cornerEventFilter(corner, event);

  return false;
}

/////////////////////////////////////////////////
bool RectItem::rotateEventFilter(RotateHandle *_rotate,
    QGraphicsSceneMouseEvent *_event)
{
  switch (_event->type())
  {
    case QEvent::GraphicsSceneMousePress:
    {
      _rotate->SetMouseState(QEvent::GraphicsSceneMousePress);
      _rotate->SetMouseDownX(_event->pos().x());
      _rotate->SetMouseDownY(_event->pos().y());

      break;
    }
    case QEvent::GraphicsSceneMouseRelease:
    {
      _rotate->SetMouseState(QEvent::GraphicsSceneMouseRelease);
      break;
    }
    case QEvent::GraphicsSceneMouseMove:
    {
      _rotate->SetMouseState(QEvent::GraphicsSceneMouseMove);
      break;
    }
    default:
      return false;
      break;
  }

  if (_rotate->GetMouseState() == QEvent::GraphicsSceneMouseMove)
  {
    QPoint localCenter(this->drawingOriginX +
      (this->drawingOriginX + this->drawingWidth)/2,
      this->drawingOriginY + (this->drawingOriginY + this->drawingHeight)/2);
    QPointF center = this->mapToScene(localCenter);

    QPointF newPoint = _event->scenePos();
    QLineF prevLine(center.x(), center.y(),
        _event->lastScenePos().x(), _event->lastScenePos().y());
    QLineF line(center.x(), center.y(), newPoint.x(), newPoint.y());

    double angle = -prevLine.angleTo(line);
    this->translate(localCenter.x(), localCenter.y());
    this->rotate(angle);
    this->translate(-localCenter.x(), -localCenter.y());

    rotationAngle += angle;
//    this->setTransformOriginPoint(localCenter);
//    this->setRotation(this->rotation() -prevLine.angleTo(line));
  }
  return true;
}

/////////////////////////////////////////////////
bool RectItem::cornerEventFilter(CornerGrabber *_corner,
    QGraphicsSceneMouseEvent *_event)
{
//  this->setTransformOriginPoint(QPointF(0,0));

  switch (_event->type())
  {
    case QEvent::GraphicsSceneMousePress:
    {
      _corner->SetMouseState(QEvent::GraphicsSceneMousePress);
      _corner->SetMouseDownX(_event->pos().x());
      _corner->SetMouseDownY(_event->pos().y());
      break;
    }
    case QEvent::GraphicsSceneMouseRelease:
    {
      _corner->SetMouseState(QEvent::GraphicsSceneMouseRelease);
      break;
    }
    case QEvent::GraphicsSceneMouseMove:
    {
      _corner->SetMouseState(QEvent::GraphicsSceneMouseMove);
      break;
    }
    default:
      return false;
      break;
  }

  if (_corner->GetMouseState() == QEvent::GraphicsSceneMouseMove)
  {
    double xPos = _event->pos().x();
    double yPos = _event->pos().y();

    // depending on which corner has been grabbed, we want to move the position
    // of the item as it grows/shrinks accordingly. so we need to either add
    // or subtract the offsets based on which corner this is.

    int xAxisSign = 0;
    int yAxisSign = 0;
    switch(_corner->GetIndex())
    {
      // corners
      case 0:
      {
        xAxisSign = 1;
        yAxisSign = 1;
        break;
      }
      case 1:
      {
        xAxisSign = -1;
        yAxisSign = 1;
        break;
      }
      case 2:
      {
        xAxisSign = -1;
        yAxisSign = -1;
        break;
      }
      case 3:
      {
        xAxisSign = +1;
        yAxisSign = -1;
        break;
      }
      //edges
      case 4:
      {
        xAxisSign = 1;
        yAxisSign = 0;
        break;
      }
      case 5:
      {
        xAxisSign = 0;
        yAxisSign = 1;
        break;
      }
      case 6:
      {
        xAxisSign = -1;
        yAxisSign = 0;
        break;
      }
      case 7:
      {
        xAxisSign = 0;
        yAxisSign = -1;
        break;
      }
      default:
        break;
    }

    // if the mouse is being dragged, calculate a new size and also position
    // for resizing the box

    double xMoved = _corner->GetMouseDownX() - xPos;
    double yMoved = _corner->GetMouseDownY() - yPos;

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
    switch(_corner->GetIndex())
    {
      case 0:
      {
        this->setPos(this->pos() +
            (_event->scenePos() - _event->lastScenePos()));
        break;
      }
      case 1:
      {
        dx = sin(-angle) * deltaHeight;
        dy = cos(-angle) * deltaHeight;
        this->setPos(this->pos() + QPointF(dx, dy));
        break;
      }
      case 3:
      {
        dx = cos(angle) * deltaWidth;
        dy = sin(angle) * deltaWidth;
        this->setPos(this->pos() + QPointF(dx, dy));

        break;
      }
      case 4:
      {
        dx = cos(angle) * deltaWidth;
        dy = sin(angle) * deltaWidth;
        this->setPos(this->pos() + QPointF(dx, dy));
        break;
      }
      case 5:
      {
        dx = sin(-angle) * deltaHeight;
        dy = cos(-angle) * deltaHeight;
        this->setPos(this->pos() + QPointF(dx, dy));
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
  _event->setAccepted(true);

  /// TODO: uncomment to enable snap to grid
/*  this->location.setX( (static_cast<int>(this->location.x())
      / this->gridSpace) * this->gridSpace);
  this->location.setY( (static_cast<int>(this->location.y())
      / this->gridSpace) * this->gridSpace);*/

  this->setPos(this->location);
}

/////////////////////////////////////////////////
void RectItem::mousePressEvent(QGraphicsSceneMouseEvent *_event)
{
  if (!this->isSelected())
    this->scene()->clearSelection();
//  QGraphicsItem::mousePressEvent(_event);
//  return;
  this->setSelected(true);
  this->location = this->pos();
  _event->setAccepted(true);
}

/////////////////////////////////////////////////
void RectItem::mouseMoveEvent(QGraphicsSceneMouseEvent *_event)
{
//  QGraphicsItem::mouseMoveEvent(_event);
//  return;

  if (!this->isSelected())
    return;

  this->location += (_event->scenePos() - _event->lastScenePos());
  this->setPos(this->location);

}

/////////////////////////////////////////////////
void RectItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *_event)
{
  _event->setAccepted(true);
}

/////////////////////////////////////////////////
void RectItem::hoverLeaveEvent(QGraphicsSceneHoverEvent *)
{
  if (!this->isSelected())
    return;

//    this->borderColor = Qt::black;
  for (int i = 0; i < 8; ++i)
    this->corners[i]->removeSceneEventFilter(this);
  this->rotateHandle->removeSceneEventFilter(this);

}

/////////////////////////////////////////////////
void RectItem::hoverEnterEvent(QGraphicsSceneHoverEvent *)
{
  if (!this->isSelected())
    return;

//    this->borderColor = Qt::red;
  for (int i = 0; i < 8; ++i)
    this->corners[i]->installSceneEventFilter(this);
  this->rotateHandle->installSceneEventFilter(this);
  this->UpdateCornerPositions();
}

/////////////////////////////////////////////////
void RectItem::UpdateCornerPositions()
{
  int cornerWidth = (this->corners[0]->boundingRect().width())/2;
  int cornerHeight = (this->corners[0]->boundingRect().height())/2;

  this->corners[0]->setPos(this->drawingOriginX - cornerWidth,
      this->drawingOriginY - cornerHeight);
  this->corners[1]->setPos(this->drawingWidth - cornerWidth,
      this->drawingOriginY - cornerHeight);
  this->corners[2]->setPos(this->drawingWidth - cornerWidth,
      this->drawingHeight - cornerHeight);
  this->corners[3]->setPos(this->drawingOriginX - cornerWidth,
      this->drawingHeight - cornerHeight);

  this->corners[4]->setPos(this->drawingOriginX - cornerWidth,
      this->drawingHeight/2 - cornerHeight);
  this->corners[5]->setPos(this->drawingWidth/2 - cornerWidth,
      this->drawingOriginY - cornerHeight);
  this->corners[6]->setPos(this->drawingWidth - cornerWidth,
      this->drawingHeight/2 - cornerHeight);
  this->corners[7]->setPos(this->drawingWidth/2 - cornerWidth,
      this->drawingHeight - cornerHeight);

  this->rotateHandle->setPos(this->drawingWidth/2,
      this->drawingOriginY);

  this->setPolygon(QPolygonF(this->boundingRect()));
}

/////////////////////////////////////////////////
void RectItem::SetWidth(int _width)
{
  this->width = _width;
  this->drawingWidth = this->width;
  this->UpdateCornerPositions();
  this->update();
}

/////////////////////////////////////////////////
void RectItem::SetHeight(int _height)
{
  this->height = _height;
  this->drawingHeight = this->height;
  this->UpdateCornerPositions();
  this->update();
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
}

/////////////////////////////////////////////////
int RectItem::GetWidth()
{
  return this->drawingWidth;
}

/////////////////////////////////////////////////
int RectItem::GetHeight()
{
  return this->drawingHeight;
}

/////////////////////////////////////////////////
QRectF RectItem::boundingRect() const
{
  return QRectF(0, 0, this->width, this->height);
}

/////////////////////////////////////////////////
void RectItem::drawBoundingBox(QPainter *_painter)
{
  _painter->save();
  QPen boundingBoxPen;
  boundingBoxPen.setStyle(Qt::SolidLine);
  boundingBoxPen.setColor(Qt::darkGray);
  _painter->setPen(boundingBoxPen);
  _painter->setOpacity(0.8);
  _painter->drawRect(this->boundingRect());
  _painter->restore();

}

/////////////////////////////////////////////////
void RectItem::paint(QPainter *_painter, const QStyleOptionGraphicsItem *,
    QWidget *)
{
  _painter->save();

  QPointF topLeft(this->drawingOriginX, this->drawingOriginY);
  QPointF topRight(this->drawingWidth, this->drawingOriginY);
  QPointF bottomLeft(this->drawingOriginX, this->drawingHeight);
  QPointF bottomRight(this->drawingWidth, this->drawingHeight);

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
