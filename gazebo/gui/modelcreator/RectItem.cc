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
#include <QDebug>
#include "math.h"
#include "CornerGrabber.hh"

/////////////////////////////////////////////////
RectItem::RectItem():
    text(),
    outterBorderColor(Qt::black),
    outterBorderPen(),
    location(0,0),
    dragStart(0,0),
    gridSpace(10),
    width(100),
    height(10),
    cornerDragStart(0,0),
    xCornerGrabBuffer(10),
    yCornerGrabBuffer(10)
{
  this->drawingOriginX = 0;
  this->drawingOriginY = 0;

  this->drawingWidth = this->width;
  this->drawingHeight = this->height;

//  this->text.setPos(35,35);
//  this->text.setPlainText("text goes here");
//  this->text.setParentItem(this);


  this->corners[0] = new CornerGrabber(this,0);
  this->corners[1] = new CornerGrabber(this,1);
  this->corners[2] = new CornerGrabber(this,2);
  this->corners[3] = new CornerGrabber(this,3);

  this->UpdateCornerPositions();

  this->setAcceptHoverEvents(true);
}

 /////////////////////////////////////////////////
RectItem::~RectItem()
{
  this->corners[0]->setParentItem(NULL);
  this->corners[1]->setParentItem(NULL);
  this->corners[2]->setParentItem(NULL);
  this->corners[3]->setParentItem(NULL);

  delete this->corners[0];
  delete this->corners[1];
  delete this->corners[2];
  delete this->corners[3];
}

/**
 *  To allow the user to grab the corners to re-size, we need to get a hover
 *  indication. But if the mouse pointer points to the left, then when the mouse
 *  tip is to the left but just outsize the box, we will not get the hover.
 *  So the solution is to tell the graphics scene the box is larger than
 *  what the painter actually paints in. This way when the user gets the mouse
 *  within a few pixels of what appears to be the edge of the box, we get
 *  the hover indication.

 *  So the cornerGrabBuffer is a few pixel wide buffer zone around the outside
 *  edge of the box.
 *
 */
/////////////////////////////////////////////////
void RectItem::AdjustSize(int _x, int _y)
{
  this->width += _x;
  this->height += _y;

//  this->drawingWidth = this->width - this->xCornerGrabBuffer;
//  this->drawingHeight = this->height - this->yCornerGrabBuffer;
  this->drawingWidth = this->width;
  this->drawingHeight = this->height;
}

/////////////////////////////////////////////////
bool RectItem::sceneEventFilter(QGraphicsItem * _watched, QEvent *_event)
{
  CornerGrabber *corner = dynamic_cast<CornerGrabber *>(_watched);
  if (corner == NULL)
    return false;

  QGraphicsSceneMouseEvent *event =
      dynamic_cast<QGraphicsSceneMouseEvent*>(_event);
  if ( event == NULL)
    return false;

  switch (event->type() )
  {
    case QEvent::GraphicsSceneMousePress:
    {
      corner->SetMouseState(CornerGrabber::kMouseDown);
      corner->SetMouseDownX(event->pos().x());
      corner->SetMouseDownY(event->pos().y());
      break;
    }
    case QEvent::GraphicsSceneMouseRelease:
    {
      corner->SetMouseState(CornerGrabber::kMouseReleased);
      break;
    }
    case QEvent::GraphicsSceneMouseMove:
    {
      corner->SetMouseState(CornerGrabber::kMouseMoving );
      break;
    }
    default:
      return false;
      break;
  }

  if ( corner->GetMouseState() == CornerGrabber::kMouseMoving )
  {
    qreal xPos = event->pos().x();
    qreal yPos = event->pos().y();

    // depending on which corner has been grabbed, we want to move the position
    // of the item as it grows/shrinks accordingly. so we need to either add
    // or subtract the offsets based on which corner this is.

    int xAxisSign = 0;
    int yAxisSign = 0;
    switch(corner->GetIndex())
    {
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
      default:
        break;
    }

    // if the mouse is being dragged, calculate a new size and also position
    // for resizing the box

    int xMoved = corner->GetMouseDownX() - xPos;
    int yMoved = corner->GetMouseDownY() - yPos;

    int newWidth = this->width + (xAxisSign * xMoved);
    if (newWidth < 20)
      newWidth  = 20;

    int newHeight = this->height + (yAxisSign * yMoved) ;
    if (newHeight < 20)
      newHeight = 20;

    int deltaWidth = newWidth - this->width;
    int deltaHeight = newHeight - this->height;

    this->AdjustSize(deltaWidth, deltaHeight);

    deltaWidth *= (-1);
    deltaHeight *= (-1);

    if (corner->GetIndex() == 0)
    {
      int newXpos = this->pos().x() + deltaWidth;
      int newYpos = this->pos().y() + deltaHeight;
      this->setPos(newXpos, newYpos);
    }
    else if (corner->GetIndex() == 1)
    {
      int newYpos = this->pos().y() + deltaHeight;
      this->setPos(this->pos().x(), newYpos);
    }
    else if (corner->GetIndex() == 3)
    {
      int newXpos = this->pos().x() + deltaWidth;
      this->setPos(newXpos,this->pos().y());
    }
    this->UpdateCornerPositions();
    this->update();
  }
  return true;
}

/////////////////////////////////////////////////
void RectItem::mouseReleaseEvent ( QGraphicsSceneMouseEvent *_event)
{
  _event->setAccepted(true);
  this->location.setX( ( static_cast<int>(this->location.x())
      / this->gridSpace) * this->gridSpace);
  this->location.setY( ( static_cast<int>(this->location.y())
      / this->gridSpace) * this->gridSpace);
  this->setPos(this->location);
}

/////////////////////////////////////////////////
void RectItem::mousePressEvent ( QGraphicsSceneMouseEvent *_event)
{
  _event->setAccepted(true);
  this->dragStart = _event->pos();
}

/////////////////////////////////////////////////
void RectItem::mouseMoveEvent ( QGraphicsSceneMouseEvent *_event)
{
  QPointF newPos = _event->pos() ;
  this->location += (newPos - this->dragStart);
  this->setPos(this->location);
}

/////////////////////////////////////////////////
void RectItem::hoverLeaveEvent (QGraphicsSceneHoverEvent *)
{
  this->outterBorderColor = Qt::black;

  this->corners[0]->removeSceneEventFilter(this);
  this->corners[1]->removeSceneEventFilter(this);
  this->corners[2]->removeSceneEventFilter(this);
  this->corners[3]->removeSceneEventFilter(this);
}

/////////////////////////////////////////////////
void RectItem::hoverEnterEvent (QGraphicsSceneHoverEvent *)
{
  this->outterBorderColor = Qt::red;

  this->corners[0]->installSceneEventFilter(this);
  this->corners[1]->installSceneEventFilter(this);
  this->corners[2]->installSceneEventFilter(this);
  this->corners[3]->installSceneEventFilter(this);

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
}

/////////////////////////////////////////////////
QRectF RectItem::boundingRect() const
{
  return QRectF(0,0,this->width,this->height);
}

// example of a drop shadow effect on a box, using QLinearGradient and two boxes
/////////////////////////////////////////////////
void RectItem::paint (QPainter *_painter, const QStyleOptionGraphicsItem *, QWidget *)
{
  QPointF topLeft(this->drawingOriginX, this->drawingOriginY);
  QPointF topRight(this->drawingWidth, this->drawingOriginY);
  QPointF bottomLeft(this->drawingOriginX, this->drawingHeight);
  QPointF bottomRight(this->drawingWidth, this->drawingHeight);

  QPointF middleLeft(this->drawingOriginX, this->drawingHeight/2.0);
  QPointF middleRight(this->drawingWidth, this->drawingHeight/2.0);

  QPen pen;
  pen.setStyle(Qt::SolidLine);
  pen.setColor(outterBorderColor);
  _painter->setPen(pen);

  _painter->drawLine(topLeft, bottomLeft);
  _painter->drawLine(topRight, bottomRight);
  _painter->drawLine(middleLeft, middleRight);
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
