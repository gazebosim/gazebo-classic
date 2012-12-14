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

#include "CornerGrabber.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
CornerGrabber::CornerGrabber(QGraphicsItem *_parent, int _index) :
  QGraphicsItem(_parent),
  index(_index),
  mouseDownX(0),
  mouseDownY(0),
  borderColor(Qt::black),
  width(8),
  height(8),
  widthGrabBuffer(10),
  heightGrabBuffer(10),
  mouseButtonState(QEvent::GraphicsSceneMouseRelease)
{
  this->setParentItem(_parent);

  this->setZValue(_parent->zValue() + 1);
//  this->setZValue(5);
  this->setAcceptHoverEvents(true);
}

/////////////////////////////////////////////////
int CornerGrabber::GetIndex()
{
  return this->index;
}

/////////////////////////////////////////////////
void CornerGrabber::SetMouseState(int _state)
{
  this->mouseButtonState = _state;
}

/////////////////////////////////////////////////
int CornerGrabber::GetMouseState()
{
  return this->mouseButtonState;
}

/////////////////////////////////////////////////
QPointF CornerGrabber::GetCenterPoint()
{
  return QPointF(pos().x() + (this->width + this->widthGrabBuffer/2),
      pos().y() + (this->height + this->heightGrabBuffer)/2);
}

/////////////////////////////////////////////////
void CornerGrabber::SetWidth(double _width)
{
  this->width = _width;
}

/////////////////////////////////////////////////
void CornerGrabber::SetHeight(double _height)
{
  this->height = _height;
}

/////////////////////////////////////////////////
double CornerGrabber::GetWidth()
{
  return this->width;
}

/////////////////////////////////////////////////
double CornerGrabber::GetHeight()
{
  return this->height;
}

/////////////////////////////////////////////////
void CornerGrabber::SetColor(QColor color)
{
  this->borderColor = color;
}

/////////////////////////////////////////////////
QColor CornerGrabber::GetColor()
{
  return this->borderColor;
}

/////////////////////////////////////////////////
void CornerGrabber::SetMouseDownX(double _x)
{
  this->mouseDownX = _x;
}

/////////////////////////////////////////////////
void CornerGrabber::SetMouseDownY(double _y)
{
  this->mouseDownY = _y;
}

/////////////////////////////////////////////////
qreal CornerGrabber::GetMouseDownX()
{
  return this->mouseDownX;
}

/////////////////////////////////////////////////
qreal CornerGrabber::GetMouseDownY()
{
  return this->mouseDownY;
}

/////////////////////////////////////////////////
void CornerGrabber::mouseMoveEvent(QGraphicsSceneDragDropEvent *_event)
{
  _event->setAccepted(false);
}

/////////////////////////////////////////////////
void CornerGrabber::mousePressEvent(QGraphicsSceneDragDropEvent *_event)
{
  _event->setAccepted(false);
}

/////////////////////////////////////////////////
void CornerGrabber::mouseReleaseEvent(QGraphicsSceneMouseEvent *_event)
{
  _event->setAccepted(true);
}

/////////////////////////////////////////////////
void CornerGrabber::mousePressEvent(QGraphicsSceneMouseEvent *_event)
{
  _event->setAccepted(false);
}

/////////////////////////////////////////////////
void CornerGrabber::mouseMoveEvent(QGraphicsSceneMouseEvent *_event)
{
  _event->setAccepted(false);
}

/////////////////////////////////////////////////
void CornerGrabber::hoverLeaveEvent(QGraphicsSceneHoverEvent *_event)
{
  this->borderColor = Qt::black;
  this->update(0, 0, this->width, this->height);
  QGraphicsItem::hoverLeaveEvent(_event);
}

/////////////////////////////////////////////////
void CornerGrabber::hoverEnterEvent(QGraphicsSceneHoverEvent *_event)
{
  this->borderColor = Qt::red;
  this->update(0, 0, this->width, this->height);
  QGraphicsItem::hoverEnterEvent(_event);
//  _event->setAccepted(false);
}

/////////////////////////////////////////////////
QRectF CornerGrabber::boundingRect() const
{
//  return QRectF(-totalWidth/2, -totalHeight/2, totalWidth, totalHeight);
  return QRectF(0, 0, this->width + this->widthGrabBuffer,
      this->height + this->heightGrabBuffer);
}

/////////////////////////////////////////////////
void CornerGrabber::paint(QPainter *_painter, const QStyleOptionGraphicsItem *,
  QWidget *)
{
  _painter->save();

  double totalWidth = this->width + this->widthGrabBuffer;
  double totalHeight = this->height + this->heightGrabBuffer;

  QPen borderPen;
  borderPen.setWidth(1);
  borderPen.setColor(this->borderColor);

  borderPen.setCapStyle(Qt::SquareCap);
  borderPen.setStyle(Qt::SolidLine);
  _painter->setPen(borderPen);

  QPointF topLeft (totalWidth/2.0 - this->width/2.0,
      totalHeight/2.0 - this->height/2.0);
  QPointF bottomRight (totalWidth/2.0 + this->width/2.0,
      totalHeight/2.0 + this->height/2.0);

  QRectF rect (topLeft, bottomRight);

  QBrush brush (Qt::SolidPattern);
  brush.setColor (this->borderColor);
  _painter->fillRect(rect, brush);
  _painter->restore();
}
